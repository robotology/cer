# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(hapticdevice QUIET)
if(hapticdevice_FOUND)
    project(cer_teleop)
    message(STATUS "hapticdevice library found => ${PROJECT_NAME} will be built")
    include_directories(${YARP_INCLUDE_DIRS} ${hapticdevice_INCLUDE_DIRS})
    add_executable(${PROJECT_NAME} main.cpp)
    target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)
endif()



