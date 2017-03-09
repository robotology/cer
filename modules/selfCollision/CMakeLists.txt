#
# Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Scalzo alessandro.scalzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(PROJECTNAME selfCollision)
project(${PROJECTNAME})

get_property(SelfCollisionLib_INCLUDE_DIRS
             TARGET SelfCollisionLib
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)

get_property(R1ModelLib_INCLUDE_DIRS
             TARGET R1ModelLib
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)

get_property(RobotModelLib_INCLUDE_DIRS
             TARGET RobotModelLib
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)
			 
include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
					${R1ModelLib_INCLUDE_DIRS}
					${RobotModelLib_INCLUDE_DIRS}
                    ${SelfCollisionLib_INCLUDE_DIRS})

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} SelfCollisionLib R1ModelLib RobotModelLib)

install(TARGETS ${PROJECTNAME} DESTINATION bin)

