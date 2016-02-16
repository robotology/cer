# Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_device(faceDisplayServer TYPE cer::dev::FaceDisplayServer INCLUDE faceDisplayServer.h)

 IF (ENABLE_faceDisplayServer)

  INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${ACE_INCLUDE_DIRS})
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${iCubDev_INCLUDE_DIRS})

  yarp_add_plugin(faceDisplayServer faceDisplayServer.cpp faceDisplayServer.h)
  TARGET_LINK_LIBRARIES(faceDisplayServer  ${YARP_LIBRARIES} ${ACE_LIBRARIES})
  yarp_install(TARGETS faceDisplayServer
           COMPONENT Runtime
           LIBRARY DESTINATION ${CER_DYNAMIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES faceDisplayServer.ini
           DESTINATION ${CER_PLUGIN_MANIFESTS_INSTALL_DIR})
        
ENDIF (ENABLE_faceDisplayServer)

