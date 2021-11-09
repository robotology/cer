# Copyright (C) 2021 IIT Istituto Italiano di Tecnologia
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(cerOdometry
                    CATEGORY device
                    TYPE cer::dev::CerOdometry
                    INCLUDE CerOdometry.h
                    DEPENDS "TARGET YARP::YARP_math"
                    DEFAULT ON)

if(ENABLE_cermod_cerOdometry)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    yarp_add_plugin(cerOdometry CerOdometry.h CerOdometry.cpp)
    add_definitions(-D_USE_MATH_DEFINES)
    target_link_libraries(cerOdometry ${YARP_LIBRARIES} ctrlLib)
    yarp_install(TARGETS cerOdometry
                 EXPORT YARP_${YARP_PLUGIN_MASTER}
                 COMPONENT Runtime
                 LIBRARY DESTINATION ${CER_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${CER_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${CER_PLUGIN_MANIFESTS_INSTALL_DIR})

endif(ENABLE_cermod_cerOdometry)
