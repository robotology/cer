# Copyright (C) 2016 iCub Facility, Istituto Italiano di Tecnologia
# Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: LGPLv2.1+

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

yarp_prepare_plugin(cer2headpose TYPE Cer2HeadPose
                                 INCLUDE cer2headpose.h
                                 CATEGORY portmonitor)

if(ENABLE_cermod_cer2headpose)

  yarp_add_plugin(cer_pm_cer2headpose
                  cer2headpose.cpp
                  cer2headpose.h)

  target_link_libraries(cer_pm_cer2headpose YARP::YARP_OS
                                            YARP::YARP_init
                                            YARP::YARP_sig)

  yarp_install(TARGETS cer_pm_cer2headpose
               EXPORT YARP_${YARP_PLUGIN_MASTER}
               COMPONENT Runtime
               LIBRARY DESTINATION ${CER_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${CER_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${CER_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
