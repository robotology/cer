# Copyright: (C) 2015 iCub Facility Istituto Italiano di Tecnologia
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Prepare a device library called "cermod".
# A test executable called "cermoddev" will also be produced.

cmake_minimum_required(VERSION 2.8.9)
project(cer)

#### Find YARP, require that YARP version is at least ICUB_REQYARP_VERSION
set(CER_REQYARP_VERSION_MAJOR "2")
set(CER_REQYARP_VERSION_MINOR "3")
set(CER_REQYARP_VERSION_PATCH "64")
set(CER_REQYARP_VERSION_TWEAK "4")

if(CER_REQYARP_VERSION_TWEAK)
    set(CER_REQYARP_VERSION ${CER_REQYARP_VERSION_MAJOR}.${CER_REQYARP_VERSION_MINOR}.${CER_REQYARP_VERSION_PATCH}.${CER_REQYARP_VERSION_TWEAK})
else()
    set(CER_REQYARP_VERSION ${CER_REQYARP_VERSION_MAJOR}.${CER_REQYARP_VERSION_MINOR}.${CER_REQYARP_VERSION_PATCH})
endif()

find_package(YARP ${CER_REQYARP_VERSION} REQUIRED)
message(STATUS "YARP is version: ${YARP_VERSION}")

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

option(COMPILE_TESTS "Compile tests" off)

add_subdirectory(cermod)
add_subdirectory(libraries/cer_kinematics)

