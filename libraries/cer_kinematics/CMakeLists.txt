# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

cmake_minimum_required(VERSION 2.8.9)
project(cer_kinematics)

find_package(YARP        REQUIRED)
find_package(ICUB        REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH}
                              ${ICUB_MODULE_PATH}
                              ${ICUBCONTRIB_MODULE_PATH})

find_package(IPOPT REQUIRED)

include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()

set(headers_private include/${PROJECT_NAME}/private/helpers.h
                    include/${PROJECT_NAME}/private/arm_common.h
                    include/${PROJECT_NAME}/private/arm_full.h
                    include/${PROJECT_NAME}/private/arm_xyz.h
                    include/${PROJECT_NAME}/private/arm_full_heave.h
                    include/${PROJECT_NAME}/private/arm_xyz_heave.h)
set(headers         include/${PROJECT_NAME}/utils.h
                    include/${PROJECT_NAME}/tripod.h
                    include/${PROJECT_NAME}/arm.h)
set(sources         src/utils.cpp
                    src/tripod.cpp
                    src/arm.cpp)

source_group("Header Files" FILES ${headers_private} ${headers})
source_group("Source Files" FILES ${sources})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${IPOPT_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

add_definitions(${IPOPT_DEFINITIONS})
if(COMPILE_KIN_TESTS)
    add_library(${PROJECT_NAME} ${headers_private} ${headers} ${sources})
else()
    add_library(${PROJECT_NAME} SHARED ${headers_private} ${headers} ${sources})
endif()
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
target_link_libraries(${PROJECT_NAME} ctrlLib iKin ${YARP_LIBRARIES} ${IPOPT_LIBRARIES})

icubcontrib_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                           DESTINATION include/${PROJECT_NAME}
                                           FILES ${headers})

option(COMPILE_KIN_TESTS "Compile CER Kinematics Tests" OFF)
if(COMPILE_KIN_TESTS)
    add_subdirectory(tests)
endif()

icubcontrib_finalize_export(${PROJECT_NAME})
icubcontrib_add_uninstall_target()

