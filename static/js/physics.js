const STATE = { DISABLE_DEACTIVATION : 4 };

class Joint {
  constructor(joint, mate1, mate2) {
    this.joint = joint;
    this.mate1 = mate1;
    this.mate2 = mate2;
  }
};

class AmmoPhysics {
  constructor(clock, parameters={}) {
    this.bodies = {}; // THREE.js mesh objects
    this.joints = [];

    this.T_ = new Ammo.btTransform(); // reusable transformation object
    this.p_ = new Ammo.btVector3(0, 0, 0);
    this.q_ = new Ammo.btQuaternion(0, 0, 0, 0);

    this.collisionConfig = new Ammo.btDefaultCollisionConfiguration();
    this.dispatcher = new Ammo.btCollisionDispatcher(this.collisionConfig);
    this.overlappingPairCache = new Ammo.btDbvtBroadphase();
    this.solver = new Ammo.btSequentialImpulseConstraintSolver(); //Ammo.btDantzigSolver();

    this.world = new Ammo.btDiscreteDynamicsWorld(this.dispatcher, this.overlappingPairCache, this.solver, this.collisionConfig);
    this.world.setGravity(new Ammo.btVector3(0, -9.8, 0));
    this.clock = clock;

    this.parameters = parameters;
    this.collideGroup = {all: 0x1};
    this.lastCollisions = {};
  }

  setCollisionGroups(collisionGroups) {
    this.collideGroup = collisionGroups;
    this.collideGroup.all = 0xFFFF;
  }

  detectCollisions() {
    const numManifolds = this.dispatcher.getNumManifolds();
    const currCollisions = {};
    for (let i = 0; i < numManifolds; i++) {
      const contactManifold = this.dispatcher.getManifoldByIndexInternal(i);
      const numContacts = contactManifold.getNumContacts();
      let realContactCount = 0;
      for (let j = 0; j < numContacts; j++) {
        const contactPoint = contactManifold.getContactPoint(j);
        const distance = contactPoint.getDistance();
        if (distance > 0.0) continue;
        realContactCount++;
        // console.log(`contact at manifestIndex: ${i} contactIndex: ${j} distance: ${distance}`);
      }
      if (realContactCount === 0) continue;

      const rigidBody0 = Ammo.castObject(contactManifold.getBody0(), Ammo.btRigidBody);
      const rigidBody1 = Ammo.castObject(contactManifold.getBody1(), Ammo.btRigidBody);

      const mesh0 = rigidBody0.threeObject;
      const mesh1 = rigidBody1.threeObject;
      if (!mesh0 && !mesh1) continue;

      const options0 = mesh0.userData.options;
      const options1 = mesh1.userData.options;

      if (options0.collideGroup === 1 || options1.collideGroup === 1) continue;

      console.log(`Collision detected:`, options0.collideGroup, options1.collideGroup);
      // run the collision callback handler of the object
      if (options0.oncollision) {
        options0.oncollision(mesh0, mesh1);
      }
      if (options1.oncollision) {
        options1.oncollision(mesh1, mesh0);
      }
      if (!currCollisions.hasOwnProperty(mesh0.uuid)) {
        currCollisions[mesh0.uuid] = {};
      }
      if (!currCollisions.hasOwnProperty(mesh1.uuid)) {
        currCollisions[mesh1.uuid] = {};
      }
      currCollisions[mesh0.uuid][mesh1.uuid] = mesh1;
      currCollisions[mesh1.uuid][mesh0.uuid] = mesh0;
    }

    for (const [src, target] of Object.entries(this.lastCollisions)) {
      if (!currCollisions.hasOwnProperty(src)) {
        for (const [uuid, mesh] of Object.entries(target)) {
          if (mesh.userData.options.onseparate) {
            mesh.userData.options.onseparate(mesh, this.bodies[src]);
          }
        }
      } else {
        const intersect = currCollisions[src];
        for (const [uuid, mesh] of Object.entries(target)) {
          if (mesh.userData.options.onseparate && !intersect.hasOwnProperty(uuid)) {
            mesh.userData.options.onseparate(mesh, this.bodies[src]);
          }
        }
      }
    }

    this.lastCollisions = currCollisions;
  }

  add(mesh, options) {
    options = options || {};
    const mass = options.mass === undefined ? 0 : options.mass;

    this.T_.setIdentity();
    this.p_.setValue(mesh.position.x, mesh.position.y, mesh.position.z);
    this.T_.setOrigin(this.p_);
    this.q_.setValue(mesh.quaternion.x, mesh.quaternion.y, mesh.quaternion.z, mesh.quaternion.w);
    this.T_.setRotation(this.q_);
    const motionState = new Ammo.btDefaultMotionState(this.T_);

    const geometry_type = options.geometry || mesh.geometry.type;
    const params = options.parameters || mesh.geometry.parameters;
    let collisionShape;
    let I, w, h, d, r, l;
    switch (geometry_type) {
      case "BoxGeometry":
        I = mass / 3;
        w = params.width * 0.5;
        h = params.height * 0.5;
        d = params.depth * 0.5;
        this.p_.setValue(w, h, d);
        collisionShape = new Ammo.btBoxShape(this.p_);
        break;
      case "CylinderGeometry":
        I = mass / 12;
        r = params.radiusTop;
        h = params.height * 0.5;
        this.p_.setValue(r, h, r);
        collisionShape = new Ammo.btCylinderShape(this.p_);
        break;
      case "SphereGeometry":
        I = mass * 2 / 5;
        r = params.radius;
        collisionShape = new Ammo.btSphereShape(r);
        break;
      default:
        console.log(`${mesh.geometry.type} is not supported at this time.`);
        return null;
    }
    collisionShape.setMargin(0.01);
    this.p_.setValue(0, 0, 0);
    collisionShape.calculateLocalInertia(mass, this.p_);

    const rigidbodyInfo = new Ammo.btRigidBodyConstructionInfo(mass, motionState, collisionShape, this.p_);
    const rigidbody = new Ammo.btRigidBody(rigidbodyInfo);
    rigidbody.setActivationState(STATE.DISABLE_DEACTIVATION);

    this.world.addRigidBody(rigidbody, options.collideGroup, options.collideWith);
    mesh.userData.rigidbody = rigidbody;
    rigidbody.threeObject = mesh; // add it back to the Ammo.js rigidbody for use in collision detection
    mesh.userData.options = options;
    if (options.requires_sync !== false) {
      this.bodies[mesh.uuid] = mesh;
    }

    Ammo.destroy(rigidbodyInfo);
    return rigidbody;
  }

  hinge(A, B, joint) {
    A = A || {};
    B = B || {};
    joint = joint || new Joint(null, A, B);

    let bodyA = A.mesh || null;
    let pivotA = A.xyz || [0, 0, 0];
    let axisA = A.axis || [0, 0, 0];
    let bodyB = B.mesh || null;
    let pivotB = B.xyz || [0, 0, 0];
    let axisB = B.axis || [0, 0, 0];

    if (bodyA && bodyB) {
      bodyA = bodyA.userData ? bodyA.userData.rigidbody : bodyA;
      bodyB = bodyB.userData ? bodyB.userData.rigidbody : bodyB;
      pivotA = new Ammo.btVector3(...pivotA);
      axisA = new Ammo.btVector3(...axisA);
      pivotB = new Ammo.btVector3(...pivotB);
      axisB = new Ammo.btVector3(...axisB);
      const hingeJoint = new Ammo.btHingeConstraint(
        bodyA, bodyB, pivotA, pivotB, axisA, axisB, true
      );
      this.world.addConstraint(hingeJoint, false);
      joint.joint = hingeJoint;
      this.joints.push(joint);
      Ammo.destroy(pivotA);
      Ammo.destroy(axisA);
      Ammo.destroy(pivotB);
      Ammo.destroy(axisB);
      return joint;
    }
    return null;
  }

  p2p(A, B, joint) {
    A = A || {};
    B = B || {};
    joint = joint || new Joint(null, A, B);

    let bodyA = A.mesh || null;
    let pivotA = A.xyz || [0, 0, 0];
    let bodyB = B.mesh || null;
    let pivotB = B.xyz || [0, 0, 0];

    if (bodyA && bodyB) {
      bodyA = bodyA.userData ? bodyA.userData.rigidbody : bodyA;
      bodyB = bodyB.userData ? bodyB.userData.rigidbody : bodyB;
      pivotA = new Ammo.btVector3(...pivotA);
      pivotB = new Ammo.btVector3(...pivotB);
      const p2pJoint = new Ammo.btPoint2PointConstraint(
        bodyA, bodyB, pivotA, pivotB
      );
      this.world.addConstraint(p2pJoint, false);
      joint.joint = p2pJoint;
      this.joints.push(joint);
      Ammo.destroy(pivotA);
      Ammo.destroy(pivotB);
      return joint;
    }
    return null;
  }

  get(mesh) {
    return mesh.userData.rigidbody;
  }

  getTransform(mesh, position, quaternion) {
    const rigidbody = mesh.userData.rigidbody;
    let motionState = rigidbody.getMotionState();
    if (motionState) {
      motionState.getWorldTransform(this.T_);
      let pos = this.T_.getOrigin();
      let quat = this.T_.getRotation();
      position.set(pos.x(), pos.y(), pos.z());
      quaternion.set(quat.x(), quat.y(), quat.z(), quat.w());
    }
  }

  syncTransform(mesh) {
    this.T_.setIdentity();
    this.p_.setValue(mesh.position.x, mesh.position.y, mesh.position.z);
    this.T_.setOrigin(this.p_);
    this.q_.setValue(mesh.quaternion.x, mesh.quaternion.y, mesh.quaternion.z, mesh.quaternion.w);
    this.T_.setRotation(this.q_);
    mesh.userData.rigidbody.setWorldTransform(this.T_);
    // if (mesh.userData.rigidbody.isKinematicObject()) {
    const motionState = mesh.userData.rigidbody.getMotionState();
    if (motionState && mesh.userData.options.mass) {
      motionState.setWorldTransform(this.T_);
    }
    // }
  }

  step(dt) {
    const deltaTime = dt || this.clock.getDelta();

    // apply physics properties
    for (const [uuid, body] of Object.entries(this.bodies)) {
      const rigidbody = body.userData.rigidbody;
      if (this.parameters.angularDamping) {
        const angvel = rigidbody.getAngularVelocity();
        this.p_.setValue(
          angvel.x() * this.parameters.angularDamping,
          angvel.y() * this.parameters.angularDamping,
          angvel.z() * this.parameters.angularDamping
        );
        rigidbody.setAngularVelocity(this.p_);
      }
    }

    this.world.stepSimulation(deltaTime, 20);

    for (const [uuid, body] of Object.entries(this.bodies)) {
      const rigidbody = body.userData.rigidbody;
      let motionState = rigidbody.getMotionState();
      if (motionState) {
        motionState.getWorldTransform(this.T_);
        const pos = this.T_.getOrigin();
        const quat = this.T_.getRotation();
        body.position.set(pos.x(), pos.y(), pos.z());
        body.quaternion.set(quat.x(), quat.y(), quat.z(), quat.w());
      }
    }

    this.detectCollisions();

    return deltaTime;
  }

  reset() {
    const joints = this.joints;
    for (const joint of joints) {
      this.world.removeConstraint(joint.joint);
    }
    this.joints = [];

    for (const [uuid, body] of Object.entries(this.bodies)) {
      this.syncTransform(body);
      this.p_.setValue(0, 0, 0);
      body.userData.rigidbody.setLinearVelocity(this.p_);
      body.userData.rigidbody.setAngularVelocity(this.p_);
    }

    for (const joint of joints) {
      const joint_type = Object.getPrototypeOf(joint.joint).constructor.name;
      Ammo.destroy(joint.joint);
      joint.joint = null;
      if (joint_type === "btHingeConstraint") {
        this.hinge(joint.mate1, joint.mate2, joint);
      } else if (joint_type === "btPoint2PointConstraint") {
        this.p2p(joint.mate1, joint.mate2, joint);
      }
    }
  }

};

export default AmmoPhysics;