#ifndef OROCOS_UNIVERSALROBOTSUR5_COMPONENT_HPP
#define OROCOS_UNIVERSALROBOTSUR5_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

class UniversalRobotsUR5 : public RTT::TaskContext{
  public:
    UniversalRobotsUR5(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

  private:
    // input ports
    RTT::InputPort<std::vector<double> > q_from_robot;
    RTT::InputPort<std::vector<double> > qdot_from_robot;
    // output ports
    RTT::OutputPort<std::vector<double> > q_to_robot;
    // joint vectors
    std::vector<double> q_std;
    std::vector<double> qdot_std;
    std::vector<double> q_cmd_std;
    // kdl
    KDL::JntArray q_kdl;
    KDL::JntArray q_kdl_cmd;
    KDL::JntArray qdot_kdl;

    KDL::Frame tool_frame;
    KDL::Chain frame_chain;
    // solvers
    KDL::ChainFkSolverPos_recursive* fk_pos;
    KDL::ChainJntToJacSolver* jnt2jac;
    KDL::ChainIkSolverVel_pinv* ik_vel;
    KDL::ChainIkSolverPos_NR* ik_pos;
    // cycle time
    double time_period;


};
#endif
