# Copyright: (C) 2015 iCub Facility
# Authors: Alberto Cardellino <alberto.cardellino@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(tripodMotionControl
                    CATEGORY device
                    TYPE cer::dev::tripodMotionControl
                    INCLUDE tripodMotionControl.h
                    WRAPPER controlboard_nws_yarp)

if(ENABLE_cermod_tripodMotionControl)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    yarp_add_plugin(tripodMotionControl tripodMotionControl.h tripodMotionControl.cpp)
    add_definitions(-D_USE_MATH_DEFINES)
    target_link_libraries(tripodMotionControl ${YARP_LIBRARIES} cer_kinematics)
    yarp_install(TARGETS tripodMotionControl
                 EXPORT YARP_${YARP_PLUGIN_MASTER}
                 COMPONENT Runtime
                 LIBRARY DESTINATION ${CER_DYNAMIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${CER_PLUGIN_MANIFESTS_INSTALL_DIR})


endif(ENABLE_cermod_tripodMotionControl)
