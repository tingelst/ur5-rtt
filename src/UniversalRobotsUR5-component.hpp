#ifndef OROCOS_UNIVERSALROBOTSUR5_COMPONENT_HPP
#define OROCOS_UNIVERSALROBOTSUR5_COMPONENT_HPP

#include <rtt/RTT.hpp>

class UniversalRobotsUR5 : public RTT::TaskContext{
  public:
    UniversalRobotsUR5(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
