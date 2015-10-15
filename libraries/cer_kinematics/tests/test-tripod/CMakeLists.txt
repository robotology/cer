# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

cmake_minimum_required(VERSION 2.8.9)
project(test-tripod)

source_group("Source Files" FILES main.cpp)
include_directories(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${cer_kinematics_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ctrlLib ${YARP_LIBRARIES} cer_kinematics)

