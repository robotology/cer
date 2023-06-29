#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia
# Author: Raffaele Colombo raffaele.colombo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

project(imagePatternRecognition)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

if(NAVIGATION_USE_ROS2)
    find_package(YARP REQUIRED COMPONENTS sig dev os math cv)
else()
    find_package(YARP REQUIRED COMPONENTS sig dev os math rosmsg cv)
endif()

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
set_property(TARGET imagePatternRecognition PROPERTY FOLDER "Modules")
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
