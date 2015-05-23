typedef btRigidBody::btRigidBodyConstructionInfo btRigidBodyCI;

btCylinderShapeX robot_shape(btVector3(.03,0.01,0.0));

// TODO: add tool body
#define BODY_COUNT 5
#define JOINT_COUNT 5

class Robot {
  float               links_mass[BODY_COUNT];
  btCollisionShape*	m_shapes[BODY_COUNT];
  btRigidBody*		m_bodies[BODY_COUNT];
  btTypedConstraint*	m_joints[JOINT_COUNT];
  
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
    m_shapes[0] = new btCylinderShapeZ(btVector3(.01,0.00,0.03)); links_mass[0] = 0.1;
    m_shapes[1] = new btCylinderShape (btVector3(.01,0.03,0.00)); links_mass[1] = 0.1;
    m_shapes[2] = new btCylinderShape (btVector3(.01,0.03,0.00)); links_mass[2] = 0.1;
    m_shapes[3] = new btCylinderShape (btVector3(.01,0.03,0.00)); links_mass[3] = 0.1;
    m_shapes[4] = new btCylinderShapeX(btVector3(.03,0.01,0.00)); links_mass[4] = 0.1;
    
    // Setup rigid bodies
    btTransform offset; offset.setIdentity();
    //offset.setOrigin(positionOffset);		

    float h0 = .093;
    float l1 = .080, l2 = .081, l3 = .172;
    
      // -- link 0
      btTransform transform;
      transform.setIdentity();
      m_bodies[0] = localCreateRigidBody(btScalar(links_mass[0]), offset * transform, m_shapes[0]);

      // -- link 1
      transform.setIdentity();
      transform.setOrigin(btVector3(btScalar(0.), btScalar(0.), btScalar(h0)));
      m_bodies[1] = localCreateRigidBody(btScalar(links_mass[1]), offset * transform, m_shapes[1]);

      // -- link 2
      transform.setIdentity();
      transform.setOrigin(btVector3(btScalar(0.), btScalar(-l1), btScalar(h0)));
      m_bodies[2] = localCreateRigidBody(btScalar(links_mass[2]), offset * transform, m_shapes[2]);

      // -- link 3
      transform.setIdentity();
      transform.setOrigin(btVector3(btScalar(0.), btScalar(-l1), btScalar(h0 + l2)));
      m_bodies[3] = localCreateRigidBody(btScalar(links_mass[3]), offset * transform, m_shapes[3]);

      // -- link 4
      transform.setIdentity();
      transform.setOrigin(btVector3(btScalar(0.), btScalar(-l1 + l3/2), btScalar(h0 + l2)));
      m_bodies[4] = localCreateRigidBody(btScalar(links_mass[4]), offset * transform, m_shapes[4]);
    
    // TODO: read manual about this
    // Setup some damping on the m_bodies
    for (int i=0; i<BODY_COUNT; ++i)
    {
    	m_bodies[i]->setDamping(0.05, 0.85);
    	m_bodies[i]->setDeactivationTime(0.8);
    	m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);
    }
    
    // Setup the constraints
    // TODO: Add limits of 90 degrees to joints
    //btTransform localA, localB;

    //for (int i=0; i<JOINT_COUNT ;i++) {
    //  localA.setIdentity(); localB.setIdentity();

    //  //if(i != 4) localB = m_bodies[i+1]->getWorldTransform().inverse() * m_bodies[i]->getWorldTransform() * localA;
    //  m_joints[i] = new btHingeConstraint(*m_bodies[i], *m_bodies[i+1], localA, localB);
    //}
  }

  ~Robot(){}

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


static struct RobotCI : public btRigidBodyCI {
  RobotCI() : btRigidBodyCI(0.046, nullptr, &robot_shape, {0, 0, 0}) {
    m_restitution = 1;
  }
} robot_ci{};

static const struct BallCI : public btRigidBodyCI {
  btSphereShape shape{0.0215};
  BallCI() : btRigidBodyCI(0.046, nullptr, &shape, {0, 0, 0}) {
    m_restitution = 0.5;
  }
} ball_ci{};

struct PlaneCI : public btRigidBodyCI {
  btStaticPlaneShape shape;
  PlaneCI(btVector3 plane_normal, btScalar plane_constant)
      : btRigidBodyCI(0, nullptr, &shape, {0, 0, 0}),
        shape{plane_normal, plane_constant} {
    m_restitution = 1.0;
  }
} ground_ci{{0, 0, 1}, 0.0};

struct World {
  // the simulation timestamp, and a clock, can be used for comparison
  unsigned int frame_number{0};
  btScalar timestamp{0};
  btClock real_clock{};

  // broadphase
  btDbvtBroadphase broadphase{};

  // config and dispatcher
  btDefaultCollisionConfiguration collision_configuration{};
  btCollisionDispatcher dispatcher{&collision_configuration};

  // the solver
  btSequentialImpulseConstraintSolver solver{};

  // the dynamics
  btDiscreteDynamicsWorld dynamics{&dispatcher, &broadphase, &solver,
                                   &collision_configuration};

  // the stuff
  btRigidBody ground_body{ground_ci};
  btRigidBody ball_body{ball_ci};
  Robot robot;

  World() {
    dynamics.setGravity({0, 0, -9.80665});
    dynamics.addRigidBody(&ground_body);
    dynamics.addRigidBody(&ball_body);
    robot.addToDynamics(&dynamics);

    // drop ball from 0.0, 0.0, 1.0
    ball_body.setWorldTransform(btTransform({0, 0, 0, 1}, {0.0, 0.0, 1.0}));
    ball_body.activate(true);

    // robot
    robot.activate();
  }

  ~World(void) {
    robot.removeBodies(&dynamics); 
    dynamics.removeRigidBody(&ball_body);
    dynamics.removeRigidBody(&ground_body);
  }

  void debug_draw() {
    dynamics.debugDrawWorld();
  }

  static constexpr float default_time_step = 1.0 / 60;
  void step(float time_step=default_time_step, int max_substeps=2, float fixed_time_step=default_time_step / 2) {
    dynamics.stepSimulation(time_step, max_substeps, fixed_time_step);
    timestamp += time_step;
    frame_number++;
  }
};

World *world;
