#include "UniversalRobotsUR5-component.hpp"
#include <rtt/Component.hpp>
#include <rtt/Activity.hpp>
#include <kdl/frames_io.hpp>
#include <iostream>
#include <cmath>

UniversalRobotsUR5::UniversalRobotsUR5(std::string const& name) :
    TaskContext(name),
    q_kdl(6),
    q_kdl_cmd(6),
    qdot_kdl(6),
    q_std(6,0),
    qdot_std(6,0),
    q_cmd_std(6,0),
    fk_pos(NULL),
    jnt2jac(NULL),
    ik_vel(NULL),
    ik_pos(NULL) {
    // setup output ports
    this->ports()->addPort("UR5DesiredJointPosition", q_to_robot);
    q_to_robot.setDataSample(q_cmd_std);
    // setup input ports
    this->ports()->addPort("UR5JointPosition", q_from_robot);
    this->ports()->addPort("UR5JointVelocity", qdot_from_robot);
    // setup properties
    this->properties()->addProperty("tool_frame", tool_frame).doc(
            "pose from mounting-plate to tool center point");

    KDL::SetToZero(q_kdl);
    KDL::SetToZero(q_kdl_cmd);
    KDL::SetToZero(qdot_kdl);

  std::cout << "UniversalRobotsUR5 constructed !" <<std::endl;
}

bool UniversalRobotsUR5::configureHook(){

    //DH Params
    double DH_A[6] = {0, -0.425, -0.39243, 0, 0, 0};
    double DH_D[6] = {0.0892, 0, 0, 0.109, 0.093, 0.082};
    double DH_ALPHA[6] = {M_PI_2, 0, 0, M_PI_2, -M_PI_2, 0};
    double DH_Q_HOME_OFFSET[6] = {0, -M_PI_2, -M_PI_2 ,-M_PI_2, M_PI_2, 0};

    frame_chain = KDL::Chain();
    // create chain
    for (unsigned int i = 0; i < 6; i++) {
        frame_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH(DH_A[i], DH_ALPHA[i], DH_D[i], 0)));
    }
    KDL::JntArray q_kdl;
    q_kdl = KDL::JntArray(6);
    // initialize position vector to home offset

    for (unsigned int i=0; i < 6; i++) {
        q_kdl(i) = DH_Q_HOME_OFFSET[i];
    }

    // create solvers for the chain: pose and jacobian solver
    fk_pos = new KDL::ChainFkSolverPos_recursive(frame_chain);
    jnt2jac = new KDL::ChainJntToJacSolver(frame_chain);
    ik_vel = new KDL::ChainIkSolverVel_pinv(frame_chain);
    ik_pos = new KDL::ChainIkSolverPos_NR(frame_chain, *(fk_pos), *(ik_vel));

    //fk->JntToCart(q_kdl, tool_frame);
    std::cout << "UniversalRobotsUR5 configured !" <<std::endl;

    return true;
}

bool UniversalRobotsUR5::startHook(){
  std::cout << "UniversalRobotsUR5 started !" <<std::endl;
  return true;
}

void UniversalRobotsUR5::updateHook(){
    std::cout << "UniversalRobotsUR5 executes updateHook !" <<std::endl;


    q_from_robot.read(q_std);
    for (unsigned int i=0; i < 6; i++) {
        q_kdl(i) = q_std[i];
    }
    fk_pos->JntToCart(q_kdl, tool_frame);

    std::cout << tool_frame.p << std::endl;
    //qdot_from_robot.read(qdot_std);
    //q_std[0] += 1.0 / 60.0;
    tool_frame.p[0] += 0.001;

    ik_pos->CartToJnt(q_kdl, tool_frame, q_kdl_cmd);


    // 1 rad/s

    //for (int i = 0; i < 6; i++) {
        //std::cout << q_std[i] << " ";
    //}
    //std::cout << std::endl;

    for (unsigned int i=0; i < 6; i++) {
        q_cmd_std[i] = q_kdl_cmd(i);
    }
    q_to_robot.write(q_cmd_std);
    //q_to_robot.write(q_cmd_std);

    //std::cout << "UniversalRobotsUR5 executes updateHook !" <<std::endl;
    //this->getActivity()->trigger();
}

void UniversalRobotsUR5::stopHook() {
  std::cout << "UniversalRobotsUR5 executes stopping !" <<std::endl;
}

void UniversalRobotsUR5::cleanupHook() {
  std::cout << "UniversalRobotsUR5 cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(UniversalRobotsUR5)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(UniversalRobotsUR5)
