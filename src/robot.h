// TODO: add tool body
#define BODY_COUNT 5
#define JOINT_COUNT 5

class Robot {
  float               links_mass[BODY_COUNT];
  btCollisionShape*	m_shapes[BODY_COUNT];
  btRigidBody*		m_bodies[BODY_COUNT];
  //btTypedConstraint*	m_joints[JOINT_COUNT];
  btTransform*	    m_transforms[JOINT_COUNT];
  float	                  thetas[JOINT_COUNT] = {};
  
  btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape) {
    bool isDynamic = (mass != 0.f);
 
    btVector3 localInertia(0,0,0);
    if (isDynamic)
    	shape->calculateLocalInertia(mass,localInertia);
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    return body;
  }

public:
  //Robot(const btVector3& positionOffset) {
  Robot() {
    // Setup geometry
    m_shapes[0] = new btCylinderShapeZ(btVector3(.01,0.00,0.03)); links_mass[0] = 0.0;
    m_shapes[1] = new btCylinderShape (btVector3(.01,0.03,0.00)); links_mass[1] = 0.0;
    m_shapes[2] = new btCylinderShape (btVector3(.01,0.03,0.00)); links_mass[2] = 0.0;
    m_shapes[3] = new btCylinderShape (btVector3(.01,0.03,0.00)); links_mass[3] = 0.0;
    m_shapes[4] = new btCylinderShapeX(btVector3(.03,0.01,0.00)); links_mass[4] = 0.0;
    
    // setup transformations
    float h0 = .093;
    float l1 = .080, l2 = .081, l3 = .172;
    m_transforms[0] = new btTransform(btQuaternion({0,0,1}, thetas[0]), {   0, 0,  0});
    m_transforms[1] = new btTransform(btQuaternion({0,1,0}, thetas[1]), {   0, 0, h0});
    m_transforms[2] = new btTransform(btQuaternion({0,1,0}, thetas[2]), { -l1, 0,  0});
    m_transforms[3] = new btTransform(btQuaternion({0,1,0}, thetas[3]), {   0, 0, l2});
    m_transforms[4] = new btTransform(btQuaternion({1,0,0}, thetas[4]), {l3/2, 0,  0});

    // Setup rigid bodies
    btTransform offset; offset.setIdentity();
    
      btTransform transform;
      transform.setIdentity();

      for (int i=0; i<BODY_COUNT ;i++) {
        transform *= *m_transforms[i];
        m_bodies[i] = localCreateRigidBody(btScalar(links_mass[i]), offset * transform, m_shapes[i]);
      }
  }

  ~Robot(){
    for(int i=0; i<BODY_COUNT ;i++) {
      delete m_shapes[i];
      delete m_bodies[i];
    }

    for(int i=0; i<JOINT_COUNT ;i++) {
      delete m_transforms[i];
    }
  }

  void addToDynamics(btDiscreteDynamicsWorld* dynamics) {
    for(int i=0; i<BODY_COUNT ;i++) {
      dynamics->addRigidBody(m_bodies[i]);
    }

    //for(int i=0; i<JOINT_COUNT ;i++) {
    //  dynamics->addConstraint(m_joints[i], true);
    //}
  }

  void activate() {
    for(int i=0; i<BODY_COUNT ;i++) {
      m_bodies[i] -> activate(true);
    }
  }

  void removeBodies(btDiscreteDynamicsWorld* dynamics) {
    for(int i=0; i<BODY_COUNT ;i++) {
      dynamics->removeRigidBody(m_bodies[i]);
    }
  }
};
