# Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

option(GAZEBO_TRIPOD_PLUGIN "Compile Gazebo Yarp Plugin to control the tripod device" OFF)

if(GAZEBO_TRIPOD_PLUGIN)
    find_package(Gazebo   REQUIRED)
    # Add Gazebo CXX flags, to support Gazebo 6 reckless dependency on C++11
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


    set(gazeboTripod_source     GazeboYarpTripod.cpp
                                GazeboTripodMotionControl.cpp
                                GazeboTripod_Implementation.cpp)

    set(gazeboTripod_headers    GazeboYarpTripod.h
                                GazeboTripodMotionControl.h )

    get_property(cer_kinematics_INCLUDE_DIRS
                 TARGET cer_kinematics
                 PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                        ${YARP_INCLUDE_DIRS}
                        ${Boost_INCLUDE_DIRS}
                        ${GAZEBO_INCLUDE_DIRS}
                        ${cer_kinematics_INCLUDE_DIRS}
                        ${CMAKE_CURRENT_SOURCE_DIR}/../cermod/tripod)

    # gazeboYarpPlugin repo should export the following info and have well-done config.cmake file
    include_directories($ENV{GAZEBO_PLUGIN_ROOT}/libraries/common/include/
                        $ENV{GAZEBO_PLUGIN_ROOT}/libraries/singleton/include/)

    link_directories(/usr/local/src/robot/gazebo/install/lib)
    link_directories(${GAZEBO_LIBRARY_DIRS})

    add_library(gazeboTripod ${gazeboTripod_source} ${gazeboTripod_headers})
    target_link_libraries(gazeboTripod gazebo_yarp_singleton cer_kinematics ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
endif()
