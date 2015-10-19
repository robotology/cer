# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

cmake_minimum_required(VERSION 2.8.9)
project(cer_kinematics-tests)

include_directories(${PROJECT_SOURCE_DIR}
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${cer_kinematics_INCLUDE_DIRS})

add_executable(test-cer_kinematics-tripod    test-cer_kinematics-tripod.cpp)
add_executable(test-cer_kinematics-arm       utils.h test-cer_kinematics-arm.cpp)
add_executable(test-cer_kinematics-stability utils.h test-cer_kinematics-stability.cpp)

target_link_libraries(test-cer_kinematics-tripod    ${YARP_LIBRARIES} ctrlLib cer_kinematics)
target_link_libraries(test-cer_kinematics-arm       ${YARP_LIBRARIES} cer_kinematics)
target_link_libraries(test-cer_kinematics-stability ${YARP_LIBRARIES} cer_kinematics)

install(TARGETS     test-cer_kinematics-tripod   
                    test-cer_kinematics-arm      
                    test-cer_kinematics-stability
        DESTINATION bin)


