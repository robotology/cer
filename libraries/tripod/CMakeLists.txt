# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

cmake_minimum_required(VERSION 2.8.9)
project(tripod)

find_package(YARP        REQUIRED)
find_package(ICUB        REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH}
                              ${ICUB_MODULE_PATH}
                              ${ICUBCONTRIB_MODULE_PATH})

find_package(IPOPT REQUIRED)

include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()

set(headers include/${PROJECT_NAME}/tripod.h)
set(sources src/tripod.cpp)

source_group("Header Files" FILES ${headers})
source_group("Source Files" FILES ${sources})

include_directories(${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
                    ${IPOPT_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

add_definitions(${IPOPT_DEFINITIONS})
add_library(${PROJECT_NAME} ${headers} ${sources})
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "d")
endif ()

target_link_libraries(${PROJECT_NAME} ctrlLib ${YARP_LIBRARIES} ${IPOPT_LIBRARIES})

icubcontrib_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
                                           EXTERNAL_INCLUDE_DIRS ${ICUB_INCLUDE_DIRS} ${YARP_LIBRARIES} ${IPOPT_INCLUDE_DIRS}
                                           DESTINATION include/${PROJECT_NAME}
                                           DEPENDS ctrlLib ${YARP_LIBRARIES} ${IPOPT_LIBRARIES}
                                           FILES ${headers})

icubcontrib_finalize_export(${PROJECT_NAME})
icubcontrib_add_uninstall_target()
