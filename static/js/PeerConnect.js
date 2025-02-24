import Peer from 'peerjs';
import * as THREE from 'three';

export const HOST = null;

class PeerConnect {
  /**
   * This class synchronizes three.js objects' physx rigidbodies with peers.
   * Any objects without rigidbodies will not be updated.
   * Assumes that all objects in the scene are already generated - no new objects will be generated.
   * All objects will be assigned a name, which is universal across peers.
   * Once a host is connected to, all updates are sent from the host to all neighboring peers.
   * When assigning the rigidbody values of an object, host values override client values.
   * If clients make underlying physics updates, then the host will have to coordinate it.
   */
  constructor(peerid, physx, onGenerate) {
    /**
     * @peerid the peerid to connect to, can be null if creating a new room
     * @physx the bullet physx engine to update with new objects
     * @onGenerate handler for when peerId is generated
     */

    this.isHost = (peerid === null);
    this.connection = peerid;
    this.physx = physx;
    this.onGenerate = onGenerate;
    this._peer_connect();

    this.tracked = {}; // [mesh_id] network trackable objects
    this.synced = {}; // [peerid (per client sync)][mesh_id] these are all children that are known to the cloud, synced per client
  }

  _peer_connect() {
    this.pc = new Peer();
    this.pc.on('open', this._on_peerjs_uuid.bind(this));
    this.pc.on('error', (err) => {
      console.log(err);
      // retry after 3 seconds
      console.log(`Retrying connection after ${3} seconds...`);
      setTimeout(this._peer_connect.bind(this), 3000);
    });
  }

  _on_peerjs_uuid(id) {
    this.peerid = id;
    if (this.connection) {
      // Create a client connection to connect to a server
      this.connection = this.pc.connect(this.connection);
      this.connection.on('open', () => {
        this.onConnect(this.connection.peer);
        this.updateTask = setInterval((() => {
          this.connection.send(JSON.stringify(this.sendData(this.connection.peer)));
        }).bind(this), 33); // 33ms = 30fps
      });
      this.connection.on('data', (data) => {
        this.onData(this.connection.peer, JSON.parse(data));
      });
    } else {
      // Create a server hook-process to wait on connections
      this.pc.on('connection', this._host_add_connection.bind(this));
      console.log(`Room is now available at peerid: ${this.peerid}`);
    }

    if (this.onGenerate) {
      this.onGenerate(this.peerid);
    }
  }

  _host_add_connection(connection) {
    this.connections = this.connections ? this.connections : {};
    this.connections[connection.peer] = connection;
    connection.on('open', () => {
      this.onConnect(connection.peer);
    });
    connection.on('data', (data) => {
      this.onData(connection.peer, JSON.parse(data));
      // send an update back
      connection.send(JSON.stringify(this.sendData(connection.peer)));
    });
  }

  ///////////////////// MESHES //////////////////////

  _getCurrentState(mesh) {
    // we will need to modify this to send more information than this
    const mesh_info = { // remember that we are working on a network, so every object has parents, children, originators
      uuid: this._uuid(mesh)
    };
    const discretize = (x) => Math.round(x * 10000) / 10000; // save on network bytes

    mesh_info.position = [
      discretize(mesh.position.x),
      discretize(mesh.position.y),
      discretize(mesh.position.z)];
    mesh_info.rotation = [
      discretize(mesh.rotation._x),
      discretize(mesh.rotation._y),
      discretize(mesh.rotation._z)]; // always Euler XYZ, see if there is a way to ensure this
    mesh_info.scale = [
      discretize(mesh.scale.x),
      discretize(mesh.scale.y),
      discretize(mesh.scale.z)];
    return mesh_info;
  }

  _update(mesh, mesh_diff) {
    // first check to make sure that the mesh can be updated
    if (mesh_diff.geometry && mesh.geometry.type !== mesh_diff.geometry.type) return false; // we will have to create a new geometry and therefore mesh
    if (mesh_diff.material && mesh.material.type !== mesh_diff.material.type) return false; // we will have to create a new material and therefore mesh

    // geometry updates are ignored for now (todo)

    // if it can be updated, update predefined properties such as color, position, rotation, and scale
    if (mesh_diff.material) {
      if (mesh_diff.material.color) {
        mesh.material.color.setRGB(...mesh_diff.material.color);
      }
    }
    if (mesh_diff.position) mesh.position.set(...mesh_diff.position);
    if (mesh_diff.rotation) mesh.rotation.set(...mesh_diff.rotation);
    if (mesh_diff.scale)    mesh.scale.set(...mesh_diff.scale);

    this.onupdatemesh(mesh);
    return true;
  }

  _diff(source, target) {
    const diff = {};
    Object.keys(source).forEach((k) => {
      // assume that B[k] and A[k] are of the same type
      const a = source[k];
      const b = target[k];
      if (b === undefined && a !== undefined) {
        diff[k] = a;
      } else {
        if (a instanceof Object) {
          const subdiff = this._diff(a, b);
          if (subdiff !== null) diff[k] = subdiff;
        } else if (a instanceof Array) {
          if (a.length !== b.length) {
            diff[k] = a;
          } else {
            for (let i = 0; i < a.length; i++) {
              if (a[i] !== b[i]) {
                diff[k] = a;
                break;
              }
            }
          }
        } else if (a !== b) {
          diff[k] = a;
        }
      }
    });
    return (Object.keys(diff).length !== 0) ? diff : null;
  }

  _merge(source, target) {
    Object.keys(source).forEach((k) => {
      // assume that B[k] and A[k] are of the same type
      const a = source[k];
      if (a instanceof Object) {
        if (!target[k]) target[k] = {};
        this._merge(a, target[k]);
      } else if (a instanceof Array) {
        if (a.length !== target[k].length) {
          target[k] = a;
        } else {
          for (let i = 0; i < a.length; i++) {
            if (a[i] !== target[k][i]) {
              target[k] = a;
              break;
            }
          }
        }
      } else if (a !== target[k]) {
        target[k] = a;
      }
    });
  }

  _find_mesh_updates(snapshot) {
    const updates = [];
    // find all parents and their rank, but only for valid tracked objects
    // we put this in update because the parent may change at times
    Object.keys(this.tracked).forEach((name) => {
      const mesh_info = this._getCurrentState(this.tracked[name]);
      if (!snapshot[name]) {
        snapshot[name] = mesh_info;
        updates.push(mesh_info);
      } else {
        // need to check if there is any difference in the mesh info
        const mesh_diff = this._diff(mesh_info, snapshot[name]);
        if (mesh_diff !== null) {
          snapshot[name] = mesh_info;
          mesh_diff.name = name;
          updates.push(mesh_info); // we only care about the diff
        }
      }
    });
    return updates;
  }

  onConnect(peerid) {
    console.log(`Now connected to ${peerid}`);
  }

  sendData(peerid) {
    /**
     * Get a scene diff, like what we do with github commits
     */
    // compare to the last known snapshot of meshes that we already updated with
    if (!this.synced[peerid]) this.synced[peerid] = {};
    const snapshot = this.synced[peerid]; // these objects are in origin UUID names
    const meshes_to_update = this._find_mesh_updates(snapshot);

    const data = {
      upd: meshes_to_update,
    };

    // push a set of differences to the sendData() hook
    return data;
  }

  onData(peerid, data) {
    // aggregate updates using scene
    if (!this.synced[peerid]) this.synced[peerid] = {};
    const snapshot = this.synced[peerid];

    const scene_meshes = {};
    this.scene.traverse((object) => {
      if (object.isMesh) {
        scene_meshes[object.uuid] = object;
      }
    });

    if (data.upd.length > 0) {
      for (const mesh_diff of data.upd) {
        if (!snapshot[mesh_diff.uuid]) {
          snapshot[mesh_diff.uuid] = mesh_diff;
        } else {
          this._merge(mesh_diff, snapshot[mesh_diff.uuid]);
        }
      }
    }
  }

  add(name, mesh) {
    mesh.userData.name = name;
    this.tracked[name] = mesh;
  }

};

export default PeerConnect;