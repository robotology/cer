#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia
# Author: Raffaele Colombo raffaele.colombo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

project(idleManager)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

find_package(YARP REQUIRED COMPONENTS sig dev os math)
include_directories(${ICUB_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})

if(DEFINED ENV{R1_OBR_BUILD_DIR})
    set(R1OBR_BUILD_PATH $ENV{R1_OBR_BUILD_DIR})
else()
    message(FATAL_ERROR "The variable R1_OBR_BUILD_DIR is not defined, make sure to add the following line to your .bashrc: export R1_OBR_BUILD_DIR=${R1_OBR_SOURCE_DIR}/${BUILD_SUFFIX} .")
endif()

target_include_directories(${PROJECT_NAME} PRIVATE ${R1OBR_BUILD_PATH}/interfaces/r1OrchestratorRPC)
target_link_directories(${PROJECT_NAME} PRIVATE ${R1OBR_BUILD_PATH}/lib)
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    ${YARP_LIBRARIES} 
    r1OrchestratorRPC) 
set_property(TARGET idleManager PROPERTY FOLDER "Modules")
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
