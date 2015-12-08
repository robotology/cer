# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Scalzo
# email:  alessandro.scalzo@iit.it

cmake_minimum_required(VERSION 2.8.9)
project(cer_kinematics_alt-tests)

get_property(cer_kinematics_alt_INCLUDE_DIRS TARGET cer_kinematics_alt PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)

include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${cer_kinematics_INCLUDE_DIRS})

if(MSVC)
  add_definitions(-D_USE_MATH_DEFINES)
endif()

add_executable(test-cer_kinematics_alt-tracking test-cer_kinematics_alt-tracking.cpp)
target_link_libraries(test-cer_kinematics_alt-tracking ${YARP_LIBRARIES} cer_kinematics_alt)
install(TARGETS test-cer_kinematics_alt-tracking DESTINATION bin)

