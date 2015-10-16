# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

cmake_minimum_required(VERSION 2.8.9)
project(cer_kinematics-tests)

find_package(YARP        REQUIRED)
find_package(ICUB        REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH}
                              ${ICUB_MODULE_PATH})

include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${cer_kinematics_INCLUDE_DIRS})

add_executable(test-tripod    test-tripod.cpp)
add_executable(test-arm       test-arm.cpp)
add_executable(test-stability test-stability.cpp)

target_link_libraries(test-tripod    ${YARP_LIBRARIES} ctrlLib  cer_kinematics)
target_link_libraries(test-arm       ${YARP_LIBRARIES} cer_kinematics)
target_link_libraries(test-stability ${YARP_LIBRARIES} cer_kinematics)

