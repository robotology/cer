# Copyright: (C) 2015 iCub Facility Istituto Italiano di Tecnologia
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Prepare a device library called "cermod".
# A test executable called "cermoddev" will also be produced.

cmake_minimum_required(VERSION 2.8.9)
project(cer)

find_package(YARP 2.3.64.4 REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

find_package(ICUB REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

find_package(IPOPT REQUIRED)

if(NOT WIN32)
    option(CER_SHARED_LIBRARY "Compile shared libraries rather than static libraries" ON)
    if(CER_SHARED_LIBRARY)
        set(BUILD_SHARED_LIBS ON)
    endif()
endif()
set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "cer is always built with dynamic plugins")
include(YarpInstallationHelpers)
yarp_configure_external_installation(cer WITH_PLUGINS)


include(AddInstallRPATHSupport)

add_install_rpath_support(LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"                                   # Libraries
                          BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"                                   # Binaries
                                   "${CMAKE_INSTALL_PREFIX}/${CER_DYNAMIC_PLUGINS_INSTALL_DIR}"     # Plugins
                          USE_LINK_PATH)

# Everything is configured now, we can start adding source code
add_subdirectory(app)
add_subdirectory(libraries/cer_kinematics)
add_subdirectory(cermod)
add_subdirectory(gazeboYarpPlugin)
add_subdirectory(modules)
