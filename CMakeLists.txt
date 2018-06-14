# Copyright: (C) 2015 iCub Facility Istituto Italiano di Tecnologia
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Prepare a device library called "cermod".
# A test executable called "cermoddev" will also be produced.

cmake_minimum_required(VERSION 2.8.9)
project(cer)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 11)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(YARP 3.0.0 COMPONENTS OS sig dev math rosmsg REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

find_package(ICUB REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

find_package(IPOPT)
if(NOT IPOPT_FOUND)
    message(WARNING "IPOPT library was not found, some kinematics module will not be available.")
endif()

if(NOT WIN32)
    option(CER_SHARED_LIBRARY "Compile shared libraries rather than static libraries" ON)
    if(CER_SHARED_LIBRARY)
        set(BUILD_SHARED_LIBS ON)
    endif()
endif()

include(GNUInstallDirs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "cer is always built with dynamic plugins")
include(YarpInstallationHelpers)
yarp_configure_external_installation(cer WITH_PLUGINS)

include(AddInstallRPATHSupport)

add_install_rpath_support(LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"                                   # Libraries
                          BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"                                   # Binaries
                                   "${CMAKE_INSTALL_PREFIX}/${CER_DYNAMIC_PLUGINS_INSTALL_DIR}"     # Plugins
                          USE_LINK_PATH)

# Everything is configured now, we can start adding source code

if(IPOPT_FOUND)
    add_subdirectory(libraries/cer_kinematics)
endif()

add_subdirectory(libraries/cer_kinematics_alt)
add_subdirectory(libraries/robot_self_collision)

add_subdirectory(app)
add_subdirectory(cermod)
add_subdirectory(gazeboYarpPlugin)
add_subdirectory(modules)
add_subdirectory(tools)

option(CER_COMPILE_TESTS "Compile tests" OFF)
if(CER_COMPILE_TESTS)
  add_subdirectory(tests)
endif()


