typedef btRigidBody::btRigidBodyConstructionInfo btRigidBodyCI;

#include "robot.h"

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
  Robot robot;

  World() {
    dynamics.setGravity({0, 0, -9.80665});
    dynamics.addRigidBody(&ground_body);
    Eigen::VectorXd angles(5);
    angles[0] = 0; angles[1] = 0;
    angles[2] = 0; angles[3] = 0;
    angles[4] = 0;

    robot.addTheta2List(angles);
    angles[0] = M_PI; robot.addTheta2List(angles);
    angles[1] = M_PI; robot.addTheta2List(angles);
    angles[2] = M_PI; robot.addTheta2List(angles);

    robot.addToDynamics(&dynamics);
    robot.activate();
  }

  ~World(void) {
    robot.removeBodies(&dynamics); 
    dynamics.removeRigidBody(&ground_body);
  }

  void debug_draw() {
    dynamics.debugDrawWorld();
  }

  static constexpr float default_time_step = 1.0 / 60;
  void step(float time_step=default_time_step, int max_substeps=2, float fixed_time_step=default_time_step / 2) {
    robot.moveRobot();

    dynamics.stepSimulation(time_step, max_substeps, fixed_time_step);
    timestamp += time_step;
    frame_number++;
  }
};

World *world;
