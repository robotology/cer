# Copyright: (C) 2015 iCub Facility
# Authors: Alberto Cardellino <alberto.cardellino@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(cerDoubleLidar
                    CATEGORY device
                    TYPE cer::dev::cerDoubleLidar
                    INCLUDE cerDoubleLidar.h
                    DEPENDS "NOT APPLE")


if(ENABLE_cermod_cerDoubleLidar)

  if(WIN32)
    set(RPLIDAR_ARCH_DIR win32)
  elseif(APPLE)
    set(RPLIDAR_ARCH_DIR macOS)
  elseif(UNIX)
    set(RPLIDAR_ARCH_DIR linux)
  endif()

  set(RPLIDAR_SDK_SRCS ${RPLIDAR_SDK_ROOT}/src/rplidar_driver.cpp
                       ${RPLIDAR_SDK_ROOT}/src/arch/${RPLIDAR_ARCH_DIR}/net_serial.cpp
                       ${RPLIDAR_SDK_ROOT}/src/arch/${RPLIDAR_ARCH_DIR}/timer.cpp
                       ${RPLIDAR_SDK_ROOT}/src/hal/thread.cpp)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    yarp_add_plugin(cerDoubleLidar cerDoubleLidar.h cerDoubleLidar.cpp)
    add_definitions(-D_USE_MATH_DEFINES)

    yarp_install(TARGETS cerDoubleLidar
                 COMPONENT Runtime
                 LIBRARY DESTINATION ${CER_DYNAMIC_PLUGINS_INSTALL_DIR})

    target_link_libraries(cerDoubleLidar ${YARP_LIBRARIES} )

    yarp_install(FILES cerDoubleLidar.ini
                 DESTINATION ${CER_PLUGIN_MANIFESTS_INSTALL_DIR})

endif(ENABLE_cermod_cerDoubleLidar)
