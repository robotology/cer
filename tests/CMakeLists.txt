# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

cmake_minimum_required(VERSION 2.8.9)
project(cer-tests)

get_property(cer_kinematics_INCLUDE_DIRS
             TARGET cer_kinematics
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)

get_property(cer_kinematics_alt_INCLUDE_DIRS
             TARGET cer_kinematics_alt
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)

include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${cer_kinematics_INCLUDE_DIRS}
                    ${cer_kinematics_alt_INCLUDE_DIRS})

add_definitions(-D_USE_MATH_DEFINES)
add_executable(test-cer_kinematics-b2b test-cer_kinematics-b2b.cpp)
target_link_libraries(test-cer_kinematics-b2b ${YARP_LIBRARIES} iKin cer_kinematics cer_kinematics_alt)
install(TARGETS test-cer_kinematics-b2b DESTINATION bin)


## Tests using RTF - Robot Testing Framework

# find a version of RTF higher then 1.0.1
# and with yarp support enabled
find_package(RTF 1.0.1 COMPONENTS YARP REQUIRED)

# set the output plugin directory to collect all the shared libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_SHARED_MODULE_PREFIX "")

# Build example?
# add_subdirectory(example)

# Build camera tests
add_subdirectory(src/tripod)

