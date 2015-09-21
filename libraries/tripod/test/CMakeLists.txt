# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

cmake_minimum_required(VERSION 2.8.9)
project(test)

find_package(YARP        REQUIRED)
find_package(ICUB        REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH}
                              ${ICUB_MODULE_PATH}
                              ${ICUBCONTRIB_MODULE_PATH})

find_package(tripod)

source_group("Source Files" FILES main.cpp)
include_directories(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${tripod_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ctrlLib ${YARP_LIBRARIES} tripod)

