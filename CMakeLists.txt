# Copyright: (C) 2015 iCub Facility Istituto Italiano di Tecnologia
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Prepare a device library called "cermod".
# A test executable called "cermoddev" will also be produced.

cmake_minimum_required(VERSION 2.8.9)
project(cer)

find_package(YARP 2.3.64.4 REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

option(COMPILE_KIN_TESTS "Compile CER Kinematics Tests" OFF)

add_subdirectory(cermod)
add_subdirectory(libraries/cer_kinematics)

