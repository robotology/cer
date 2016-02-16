# Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_device(faceDisplayClient TYPE cer::dev::FaceDisplayClient INCLUDE faceDisplayClient.h)

 IF (ENABLE_faceDisplayClient)

	INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${ACE_INCLUDE_DIRS})
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${iCubDev_INCLUDE_DIRS})

	yarp_add_plugin(faceDisplayClient faceDisplayClient.cpp faceDisplayClient.h)
	TARGET_LINK_LIBRARIES(faceDisplayClient  ${YARP_LIBRARIES} ${ACE_LIBRARIES})
	yarp_install(TARGETS faceDisplayClient
	         COMPONENT Runtime
	         LIBRARY DESTINATION ${CER_DYNAMIC_PLUGINS_INSTALL_DIR})

	yarp_install(FILES faceDisplayClient.ini
	         DESTINATION ${CER_PLUGIN_MANIFESTS_INSTALL_DIR})
        
ENDIF (ENABLE_faceDisplayClient)

