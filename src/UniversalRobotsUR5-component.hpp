#ifndef OROCOS_UNIVERSALROBOTSUR5_COMPONENT_HPP
#define OROCOS_UNIVERSALROBOTSUR5_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

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
    KDL::Frame tool_frame;
    KDL::JntArray q_kdl;
    KDL::Chain frame_chain;
    KDL::ChainFkSolverPos_recursive* fk;
    KDL::ChainJntToJacSolver* jnt2jac;
    // cycle time
    double time_period;


};
#endif
