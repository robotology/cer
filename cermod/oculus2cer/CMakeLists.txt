# Copyright (C) 2016 iCub Facility, Istituto Italiano di Tecnologia
# Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: LGPLv2.1+

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

yarp_prepare_plugin(oculus2cer TYPE Oculus2Cer
                               INCLUDE oculus2cer.h
                               CATEGORY portmonitor)

if(ENABLE_cermod_oculus2cer)
  include_directories(${YARP_INCLUDE_DIRS})

  yarp_install(FILES oculus2cer.ini
               COMPONENT runtime
               DESTINATION ${CER_PLUGIN_MANIFESTS_INSTALL_DIR})

  yarp_add_plugin(cer_pm_oculus2cer
                  oculus2cer.cpp
                  oculus2cer.h)

  target_link_libraries(cer_pm_oculus2cer YARP::YARP_OS
                                           YARP::YARP_init)

  yarp_install(TARGETS cer_pm_oculus2cer
               COMPONENT Runtime
               LIBRARY DESTINATION ${CER_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${CER_STATIC_PLUGINS_INSTALL_DIR})

endif()
