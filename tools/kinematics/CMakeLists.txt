# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

cmake_minimum_required(VERSION 3.5)
project(cer_kinematics-tools)

add_definitions(-D_USE_MATH_DEFINES)
add_executable(cer_kinematics-tripod    cer_kinematics-tripod.cpp)
add_executable(cer_kinematics-forward   cer_kinematics-forward.cpp)
add_executable(cer_kinematics-arm       cer_kinematics-arm.cpp)
add_executable(cer_kinematics-head      cer_kinematics-head.cpp)
add_executable(cer_kinematics-stability cer_kinematics-stability.cpp)
add_executable(cer_kinematics-tracking  cer_kinematics-tracking.cpp)
#add_executable(cer_kinematics-b2b       cer_kinematics-b2b.cpp)

target_link_libraries(cer_kinematics-tripod    ${YARP_LIBRARIES} ctrlLib cer_kinematics)
target_link_libraries(cer_kinematics-forward   ${YARP_LIBRARIES} cer_kinematics)
target_link_libraries(cer_kinematics-arm       ${YARP_LIBRARIES} cer_kinematics)
target_link_libraries(cer_kinematics-head      ${YARP_LIBRARIES} cer_kinematics)
target_link_libraries(cer_kinematics-stability ${YARP_LIBRARIES} cer_kinematics)
target_link_libraries(cer_kinematics-tracking  ${YARP_LIBRARIES} cer_kinematics)
#target_link_libraries(cer_kinematics-b2b       ${YARP_LIBRARIES} iKin cer_kinematics cer_kinematics_alt)

set_target_properties(cer_kinematics-tripod    PROPERTIES FOLDER ${PROJECT_NAME})
set_target_properties(cer_kinematics-forward   PROPERTIES FOLDER ${PROJECT_NAME})
set_target_properties(cer_kinematics-arm       PROPERTIES FOLDER ${PROJECT_NAME})
set_target_properties(cer_kinematics-head      PROPERTIES FOLDER ${PROJECT_NAME})
set_target_properties(cer_kinematics-stability PROPERTIES FOLDER ${PROJECT_NAME})
set_target_properties(cer_kinematics-tracking  PROPERTIES FOLDER ${PROJECT_NAME})
#set_target_properties(cer_kinematics-b2b       PROPERTIES FOLDER ${PROJECT_NAME})

install(TARGETS cer_kinematics-tripod
                cer_kinematics-forward
                cer_kinematics-arm
                cer_kinematics-head
                cer_kinematics-stability
                cer_kinematics-tracking
#                cer_kinematics-b2b
        DESTINATION bin)
