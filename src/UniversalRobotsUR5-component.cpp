#include "UniversalRobotsUR5-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

UniversalRobotsUR5::UniversalRobotsUR5(std::string const& name) : TaskContext(name){
  std::cout << "UniversalRobotsUR5 constructed !" <<std::endl;
}

bool UniversalRobotsUR5::configureHook(){
  std::cout << "UniversalRobotsUR5 configured !" <<std::endl;
  return true;
}

bool UniversalRobotsUR5::startHook(){
  std::cout << "UniversalRobotsUR5 started !" <<std::endl;
  return true;
}

void UniversalRobotsUR5::updateHook(){
  std::cout << "UniversalRobotsUR5 executes updateHook !" <<std::endl;
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
