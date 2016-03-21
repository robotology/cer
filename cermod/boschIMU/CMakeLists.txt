# Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_device(boschIMU TYPE yarp::dev::BoschIMU INCLUDE boschIMU.h)

 IF (ENABLE_boschIMU)

    if(WIN32)
        message("The device IMU BOSCH is not available on WINDOWS now, please un select it")
    else()
        INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${ACE_INCLUDE_DIRS})
        INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

        yarp_add_plugin(boschIMU boschIMU.cpp boschIMU.h)
        TARGET_LINK_LIBRARIES(boschIMU  ${YARP_LIBRARIES} )
        yarp_install( TARGETS boschIMU
                        COMPONENT Runtime
                        LIBRARY DESTINATION ${CER_DYNAMIC_PLUGINS_INSTALL_DIR})

        yarp_install( FILES boschIMU.ini
                        DESTINATION ${CER_PLUGIN_MANIFESTS_INSTALL_DIR})
    endif()
ENDIF (ENABLE_boschIMU)

