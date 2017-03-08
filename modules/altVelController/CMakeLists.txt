# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Scalzo
# email:  alessandro.scalzo@iit.it

project(altVelController)

get_property(RobotControlLib_INCLUDE_DIRS
             TARGET RobotControlLib
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)

get_property(R1ControlLib_INCLUDE_DIRS
             TARGET R1ControlLib
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)

get_property(R1ModelLib_INCLUDE_DIRS
             TARGET R1ModelLib
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)

get_property(RobotModelLib_INCLUDE_DIRS
             TARGET RobotModelLib
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)
			 
include_directories(${YARP_INCLUDE_DIRS}
                    ${SDL_INCLUDE_DIR}
                    ${R1ControlLib_INCLUDE_DIRS}
					${RobotControlLib_INCLUDE_DIRS}
                    ${R1ModelLib_INCLUDE_DIRS}
                    ${RobotModelLib_INCLUDE_DIRS})

add_definitions(-D_USE_MATH_DEFINES)

add_executable(${PROJECT_NAME} VelocityController.cpp)

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${SDL_LIBRARY} R1ControlLib)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
