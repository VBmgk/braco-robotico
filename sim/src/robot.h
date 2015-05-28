// TODO: add tool body
#include <iostream>
#define BODY_COUNT 5
#define JOINT_COUNT 5

#define N_SUB_THETAS 200
#define THETA_TOL M_PI/180

class Robot {
  float               links_mass[BODY_COUNT];
  btCollisionShape*   m_shapes[BODY_COUNT];
  btRigidBody*        m_bodies[BODY_COUNT];
  btTransform         m_transforms[JOINT_COUNT];
  btTransform         offset;
  Eigen::VectorXd     theta;
  std::vector< Eigen::VectorXd> theta_list;

  size_t curr_list_pos = 0;
  bool reach_final_theta = true;

  btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape) {
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
  Robot() {
    theta = Eigen::VectorXd(JOINT_COUNT);

    // Setup geometry
    m_shapes[0] = new btCylinderShapeZ(btVector3(.01,0.00,0.03)); links_mass[0] = 0.0;
    m_shapes[1] = new btCylinderShape (btVector3(.01,0.03,0.00)); links_mass[1] = 0.0;
    m_shapes[2] = new btCylinderShape (btVector3(.01,0.03,0.00)); links_mass[2] = 0.0;
    m_shapes[3] = new btCylinderShape (btVector3(.01,0.03,0.00)); links_mass[3] = 0.0;
    m_shapes[4] = new btCylinderShapeX(btVector3(.03,0.01,0.00)); links_mass[4] = 0.0;
  }

  void setupTransforms() {
    float h0 = .093, l1 = .080, l2 = .081, l3 = .172;

    m_transforms[0] = btTransform(btQuaternion({0,0,1}, theta[0]), {   0, 0,  0});
    m_transforms[1] = btTransform(btQuaternion({0,1,0}, theta[1]), {   0, 0, h0});
    m_transforms[2] = btTransform(btQuaternion({0,1,0}, theta[2]), { -l1, 0,  0});
    m_transforms[3] = btTransform(btQuaternion({0,1,0}, theta[3]), {   0, 0, l2});
    m_transforms[4] = btTransform(btQuaternion({1,0,0}, theta[4]), {l3/2, 0,  0});
  }

  void addRigidBodies(btTransform& _offset) {
    setupTransforms();

    offset = _offset;
    // Setup rigid bodies
    btTransform transform;
    transform = offset;

    for (int i=0; i<BODY_COUNT ;i++) {
      transform *= m_transforms[i];

      m_bodies[i] = localCreateRigidBody(btScalar(links_mass[i]), transform, m_shapes[i]);
    }
  }

  void moveJoint(int joint_num, float delta_ang) {
    float new_ang = theta[joint_num] + delta_ang;

    if     (new_ang > M_PI) { new_ang = M_PI; }
    else if(new_ang < 0)    { new_ang = 0;    }

    theta[joint_num] = new_ang;
  }

  void addTheta2List(Eigen::VectorXd new_theta){
    theta_list.push_back(new_theta);
    reach_final_theta = false;

    // initial position
    if(theta_list.size() == 1) theta = theta_list.at(0);
  }

  void updateState(){
    if (theta_list.size() == 0) return;

    theta +=
      (theta_list.at(curr_list_pos + 1)
       -   theta_list.at(curr_list_pos)) * (1.0/N_SUB_THETAS);

    Eigen::VectorXd buff = theta - theta_list.at(curr_list_pos + 1);
    std::cout << buff << "curr: " << curr_list_pos << std::endl;

    if (std::max( std::fabs(buff.maxCoeff()),
                  std::fabs(buff.minCoeff()) ) < THETA_TOL) {
      reach_final_theta = (++curr_list_pos == (theta_list.size()-1));
    }
  }

  void moveRobot() {
    if( !reach_final_theta) updateState();

    setupTransforms();

    btTransform transform;
    transform = offset;

    for (int i=0; i<BODY_COUNT ;i++) {
      transform *= m_transforms[i];

      m_bodies[i]->setWorldTransform(transform);
    }
  }

  ~Robot(){
    for(int i=0; i<BODY_COUNT ;i++) {
      delete m_shapes[i];
      delete m_bodies[i];
    }
  }

  void addToDynamics(btDiscreteDynamicsWorld* dynamics) {
    btTransform offset; offset.setIdentity();

    // setup transformations
    addRigidBodies(offset);
    for(int i=0; i<BODY_COUNT ;i++) {
      dynamics->addRigidBody(m_bodies[i]);
    }
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
