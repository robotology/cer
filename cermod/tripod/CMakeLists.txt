# Copyright: (C) 2015 iCub Facility
# Authors: Alberto Cardellino <alberto.cardellino@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_device(tripodMotionControl TYPE icub::dev::tripodMotionControl INCLUDE tripodMotionControl.h WRAPPER controlboardwrapper2)

if(ENABLE_cermod_tripodMotionControl)

message(INFO "compiling tripod plugin")
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})

message("tripod CMAKE_CURRENT_SOURCE_DIR is ${CMAKE_CURRENT_SOURCE_DIR}")

yarp_add_plugin(tripodMotionControl tripodMotionControl.h tripodMotionControl.cpp)
target_link_libraries(tripodMotionControl ${YARP_LIBRARIES})
yarp_install(TARGETS tripodMotionControl
             COMPONENT Runtime
             LIBRARY DESTINATION ${CER_DYNAMIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES tripodMotionControl.ini  DESTINATION ${CER_PLUGIN_MANIFESTS_INSTALL_DIR})

endif(ENABLE_cermod_tripodMotionControl)
