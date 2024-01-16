#
# Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#


set(PROJECTNAME robotJoystickControl)
project(${PROJECTNAME})

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${GSL_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

if(YARP_rosmsg_FOUND)
    add_definitions(-DROS_MSG=1)
    target_link_libraries(${PROJECTNAME} ctrlLib ${GSL_LIBRARIES} YARP::YARP_rosmsg ${YARP_LIBRARIES})
else()
    target_link_libraries(${PROJECTNAME} ctrlLib ${GSL_LIBRARIES} ${YARP_LIBRARIES})
endif()

install(TARGETS ${PROJECTNAME} DESTINATION bin)

