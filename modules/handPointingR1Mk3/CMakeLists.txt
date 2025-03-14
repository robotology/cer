#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia
# Author: Raffaele Colombo raffaele.colombo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

project(handPointingR1Mk3)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

find_package(gb-ergocub-cartesian-service)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

if(NAVIGATION_USE_ROS2)
    find_package(YARP REQUIRED COMPONENTS sig dev os math)
else()
    find_package(YARP REQUIRED COMPONENTS sig dev os math)
endif()
include_directories(${ICUB_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${gb-ergocub-cartesian-service_LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Modules")
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
