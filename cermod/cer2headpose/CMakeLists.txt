# Copyright (C) 2016 iCub Facility, Istituto Italiano di Tecnologia
# Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: LGPLv2.1+

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

yarp_prepare_plugin(cer2headpose TYPE Cer2HeadPose
                                 INCLUDE cer2headpose.h
                                 CATEGORY portmonitor)

if(ENABLE_cermod_cer2headpose)
  include_directories(${YARP_INCLUDE_DIRS})

  yarp_install(FILES cer2headpose.ini
               COMPONENT runtime
               DESTINATION ${CER_PLUGIN_MANIFESTS_INSTALL_DIR})

  yarp_add_plugin(cer_pm_cer2headpose
                  cer2headpose.cpp
                  cer2headpose.h)

  target_link_libraries(cer_pm_cer2headpose YARP::YARP_OS
                                            YARP::YARP_init
                                            YARP::YARP_sig)

  yarp_install(TARGETS cer_pm_cer2headpose
               COMPONENT Runtime
               LIBRARY DESTINATION ${CER_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${CER_STATIC_PLUGINS_INSTALL_DIR})

endif()
