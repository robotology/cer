# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(cer_teleop)
set(CMAKE_CXX_STANDARD 11)


include_directories(${PROJECT_SOURCE_DIR}/ros_messages ${PROJECT_SOURCE_DIR})
add_executable(${PROJECT_NAME} main.cpp handThread.cpp ParamParser.h)
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
if(YARP_rosmsg_FOUND)
    add_definitions(-DROS_MSG=1)
    target_link_libraries(${PROJECT_NAME} YARP::YARP_rosmsg ${YARP_LIBRARIES})
else()
    target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})
endif()
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

