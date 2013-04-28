
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libUniversalRobotsUR5-<target>.so
# and installs in the directory lib/orocos/UniversalRobotsUR5/
#
orocos_component(UniversalRobotsUR5 UniversalRobotsUR5-component.hpp UniversalRobotsUR5-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/UniversalRobotsUR5/ directory
#
orocos_install_headers(UniversalRobotsUR5-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libUniversalRobotsUR5-plugin-<target>.so
# and installs in the directory lib/orocos/UniversalRobotsUR5/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(UniversalRobotsUR5-plugin UniversalRobotsUR5-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libUniversalRobotsUR5-service-<target>.so
# and installs in the directory lib/orocos/UniversalRobotsUR5/plugins/
#
orocos_service(UniversalRobotsUR5-service UniversalRobotsUR5-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libUniversalRobotsUR5-types-<target>.so
# and installs in the directory lib/orocos/@target@/UniversalRobotsUR5/types/
#
# The header will go in include/orocos/UniversalRobotsUR5/types/UniversalRobotsUR5/UniversalRobotsUR5-types.hpp
# So you can #include <UniversalRobotsUR5/UniversalRobotsUR5-types.hpp>
#
orocos_typegen_headers(include/UniversalRobotsUR5/UniversalRobotsUR5-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !


#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


