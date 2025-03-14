# Copyright: (C) 2015 iCub Facility Istituto Italiano di Tecnologia
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Prepare a device library called "cermod".
# A test executable called "cermoddev" will also be produced.

cmake_minimum_required(VERSION 3.5)
project(CER LANGUAGES CXX C VERSION 1.0.0)

set(VARS_PREFIX "CER")

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 11)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(YCM)
find_package(YARP 3.11.100 REQUIRED COMPONENTS os sig dev math idl_tools)

find_package(ICUB REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

find_package(IPOPT)
if(NOT IPOPT_FOUND)
    message(WARNING "IPOPT library was not found, some kinematics module will not be available.")
endif()

find_package(gb-ergocub-cartesian-service REQUIRED)

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

set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "${PROJECT_NAME} is always built with dynamic plugins")
yarp_configure_external_installation(${PROJECT_NAME} WITH_PLUGINS)

include(AddInstallRPATHSupport)

add_install_rpath_support(LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"                                   # Libraries
                          BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"                                   # Binaries
                                   "${CMAKE_INSTALL_PREFIX}/${CER_DYNAMIC_PLUGINS_INSTALL_DIR}"     # Plugins
                          USE_LINK_PATH)

# Everything is configured now, we can start adding source code

if(IPOPT_FOUND)
    add_subdirectory(libraries/cer_kinematics)
    add_subdirectory(libraries/cer_mobile_kinematics)
endif()

add_subdirectory(libraries/cer_kinematics_alt)
add_subdirectory(libraries/robot_self_collision)

add_subdirectory(app)
add_subdirectory(cermod)
add_subdirectory(modules)
add_subdirectory(tools)

option(CER_COMPILE_TESTS "Compile tests" OFF)
if(CER_COMPILE_TESTS)
  add_subdirectory(tests)
endif()

include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME} VARS_PREFIX ${VARS_PREFIX}
                                            VERSION ${${VARS_PREFIX}_VERSION}
                                            COMPATIBILITY AnyNewerVersion
                                            TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                            DEPENDENCIES "YARP 3.3.0 REQUIRED COMPONENTS os sig dev math idl_tools" "ICUB REQUIRED"
                                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)

