# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

cmake_minimum_required(VERSION 2.8.9)
project(cer_kinematics-tests)

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
add_executable(test-cer_kinematics-tripod       test-cer_kinematics-tripod.cpp)
add_executable(test-cer_kinematics-forward      test-cer_kinematics-forward.cpp)
add_executable(test-cer_kinematics-arm          test-cer_kinematics-arm.cpp)
add_executable(test-cer_kinematics-head         test-cer_kinematics-head.cpp)
add_executable(test-cer_kinematics-stability    test-cer_kinematics-stability.cpp)
add_executable(test-cer_kinematics-tracking     test-cer_kinematics-tracking.cpp)
add_executable(test-cer_kinematics_alt-tracking test-cer_kinematics_alt-tracking.cpp)
add_executable(test-cer_kinematics-b2b          test-cer_kinematics-b2b.cpp)

target_link_libraries(test-cer_kinematics-tripod       ${YARP_LIBRARIES} ctrlLib cer_kinematics)
target_link_libraries(test-cer_kinematics-forward      ${YARP_LIBRARIES} cer_kinematics)
target_link_libraries(test-cer_kinematics-arm          ${YARP_LIBRARIES} cer_kinematics)
target_link_libraries(test-cer_kinematics-head         ${YARP_LIBRARIES} cer_kinematics)
target_link_libraries(test-cer_kinematics-stability    ${YARP_LIBRARIES} cer_kinematics)
target_link_libraries(test-cer_kinematics-tracking     ${YARP_LIBRARIES} cer_kinematics)
target_link_libraries(test-cer_kinematics_alt-tracking ${YARP_LIBRARIES} cer_kinematics_alt)
target_link_libraries(test-cer_kinematics-b2b          ${YARP_LIBRARIES} iKin cer_kinematics cer_kinematics_alt)

install(TARGETS     test-cer_kinematics-tripod
                    test-cer_kinematics-forward
                    test-cer_kinematics-arm
                    test-cer_kinematics-head
                    test-cer_kinematics-stability
                    test-cer_kinematics-tracking
                    test-cer_kinematics_alt-tracking
                    test-cer_kinematics-b2b
        DESTINATION bin)
