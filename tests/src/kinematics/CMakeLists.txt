# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

cmake_minimum_required(VERSION 2.8.9)
set(TESTNAME test-cer_kinematics-b2b)

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
add_executable(${TESTNAME} test-cer_kinematics-b2b.cpp)
target_link_libraries(${TESTNAME} ${YARP_LIBRARIES} iKin cer_kinematics cer_kinematics_alt)
install(TARGETS ${TESTNAME} DESTINATION bin)

