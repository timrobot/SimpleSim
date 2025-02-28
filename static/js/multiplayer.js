import * as THREE from 'three';

export const HOST = null;

class NetworkMultiplayer {
  /**
   * This class synchronizes three.js objects' physx rigidbodies with peers.
   * Any objects without rigidbodies will not be updated.
   * Assumes that all objects in the scene are already generated - no new objects will be generated.
   * All objects will be assigned a name, which is universal across peers.
   * Once a host is connected to, all updates are sent from the host to all neighboring peers.
   * When assigning the rigidbody values of an object, because host is projecting global state, it overrides client.
   * However, if a client makes a physics update, then the host must honor it.
   *  Objects are locked for that particular object, until the event gets deregistered.
   */
  constructor(peerid, physx, onGenerate) {
    /**
     * @peerid the peerid to connect to, can be null if creating a new room
     * @physx the bullet physx engine to update with new objects
     * @onGenerate handler for when peerId is generated
     */

    this.isHost = (peerid === HOST);
    this.connectionid = peerid;
    this.connection = null;
    this.physx = physx;
    this.peerid = null;

    this.tracked = {};
    for (const [uuid, body] of Object.entries(physx.bodies)) {
      if (!body.userData.options.networkName) continue;
      // network owner can be
      // - null: no one owns it, but host updates their positions
      // - 'this': the object is locked to the current owner - if the network attempts to lock with a lock request,
      //   it is first sent to the host, where it may end up being denied
      // - '<peerid>': the object is locked to the other peerid - a cached state which may be overriden once the
      //   host updates the reservation to null
      // premature ownership can happen when an object is locked on the current user, however in the future is
      // revoked and therefore reset to the host's information of where the object is located
      body.userData.networkOwner = body.userData.options.networkOwner || null;
      this.tracked[body.userData.options.networkName] = body;
    }

    this.onGenerate = onGenerate;
    this._peer_connect();

    // useable for querying and setting information on rigidbodies
    this.T_ = new Ammo.btTransform(); // reusable transformation object
    this.p_ = new Ammo.btVector3(0, 0, 0);
    this.q_ = new Ammo.btQuaternion(0, 0, 0, 0);
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
    if (this.connectionid) {
      // Create a client connection to connect to a server
      this.connection = this.pc.connect(this.connectionid);
      this.connection.on('open', () => {
        this.onConnect(this.connection.peer);
        this.updateTask = setInterval((() => {
          this.connection.send(JSON.stringify(this.sendData(this.connection.peer)));
        }).bind(this), 16); // 16ms = 60fps
      });
      this.connection.on('data', (data) => {
        this.onData(this.connection.peer, JSON.parse(data));
      });
    } else {
      // Create a server hook-process to wait on connections
      this.pc.on('connection', this._host_add_connection.bind(this));
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

  ///////////////////// PHYSX UPDATE //////////////////////

  onConnect(peerid) {
    console.log(`Now connected to ${peerid}`);
  }

  sendData(peerid) {
    /**
     * This will include things like
     * mesh.userData.rigidbody.getMotionState().getWorldTransform(this._local_transform)
     *   this._local_transform.getOrigin() -> Ammo.btVector3
     *   this._local_transform.getRotation() -> Ammo.btQuarternion
     * mesh.userData.rigidbody.getLinearVelocity() -> Ammo.btVector3
     * mesh.userData.rigidbody.getAngularVelocity() -> Ammo.btVector3
     */

    const discretize = (x) => Math.round(x * 10000) / 10000; // save on network bytes and to prevent NaN

    const data = {};
    for (const [name, mesh] of Object.entries(this.tracked)) {
      if (mesh.userData.networkOwner === 'this' || (this.isHost && mesh.userData.networkOwner !== peerid)) {
        const rigidbody = mesh.userData.rigidbody;
        const pos = mesh.position.clone();
        const quat = mesh.quaternion.clone();
        const linvel = rigidbody.getLinearVelocity();
        const angvel = rigidbody.getAngularVelocity();
        data[name] = {
          position: [discretize(pos.x), discretize(pos.y), discretize(pos.z)],
          quaternion: [discretize(quat.x), discretize(quat.y), discretize(quat.z), discretize(quat.w)],
          linearVelocity: [discretize(linvel.x()), discretize(linvel.y()), discretize(linvel.z())],
          angularVelocity: [discretize(angvel.x()), discretize(angvel.y()), discretize(angvel.z())],
          // even if we know that its been reserved, we want to indicate our request to have it null
          networkOwner: ((mesh.userData.networkOwner === 'this') ? this.peerid : (this.isHost ? mesh.userData.networkOwner : null))
        };
      } else {
        data[name] = { networkOwner: null };
      }
    }

    return data;
  }

  onData(peerid, data) {
    /**
     * We will update the value if the ownership command has been approved
     */
    for (const [name, item] of Object.entries(data)) {
      const mesh = this.tracked[name];
      if (item.hasOwnProperty('networkOwner')) {
        if ((!this.isHost && item.networkOwner !== null   && item.networkOwner !== this.peerid) ||          // host.acquire()/.update()
            (!this.isHost && item.networkOwner === null   && mesh.userData.networkOwner !== 'this') ||      // host.release()
            ( this.isHost && item.networkOwner === peerid && mesh.userData.networkOwner === null) ||        // client.acquire()
            ( this.isHost && item.networkOwner === peerid && mesh.userData.networkOwner === peerid) ||      // client.update()
            ( this.isHost && item.networkOwner === null   && mesh.userData.networkOwner === peerid)) {      // client.release()
          mesh.userData.networkOwner = item.networkOwner;
          const rigidbody = mesh.userData.rigidbody;
          if (item.hasOwnProperty('position')) { 
            mesh.position.set(...item.position);
            mesh.quaternion.copy(new THREE.Quaternion(...item.quaternion));
            this.T_.setIdentity();
            this.p_.setValue(...item.position);
            this.T_.setOrigin(this.p_);
            this.q_.setValue(...item.quaternion);
            this.T_.setRotation(this.q_);
            rigidbody.setWorldTransform(this.T_);
            this.p_.setValue(0, 0, 0);
            rigidbody.setLinearVelocity(this.p_);
            this.p_.setValue(0, 0, 0);
            rigidbody.setAngularVelocity(this.p_);
          }
        }
      }
    }
  }

};

export default NetworkMultiplayer;