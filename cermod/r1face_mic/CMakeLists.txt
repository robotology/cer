# Copyright: (C) 2018 iCub Facility
# Authors: Alberto Cardellino <alberto.cardellino@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# interface to use: IAudioGrabberSound

yarp_prepare_plugin(r1face_mic
                    CATEGORY device
                    TYPE cer::dev::R1faceMic
                    INCLUDE r1face_mic.h
                    EXTRA_CONFIG WRAPPER=AudioRecorderWrapper)

if(ENABLE_cermod_r1face_mic)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    yarp_add_plugin(r1face_mic r1face_mic.h r1face_mic.cpp)

    target_link_libraries(r1face_mic ${YARP_LIBRARIES})

    yarp_install(TARGETS r1face_mic
                 EXPORT YARP_${YARP_PLUGIN_MASTER}
                 COMPONENT Runtime
                 LIBRARY DESTINATION ${CER_DYNAMIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${CER_PLUGIN_MANIFESTS_INSTALL_DIR})


endif(ENABLE_cermod_r1face_mic)

