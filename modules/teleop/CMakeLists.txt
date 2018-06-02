# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(cer_teleop)
set(CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/ros_messages
                    ${PROJECT_SOURCE_DIR})
add_definitions(-D_USE_MATH_DEFINES)
add_executable(${PROJECT_NAME} main.cpp handThread.cpp ParamParser.h)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

