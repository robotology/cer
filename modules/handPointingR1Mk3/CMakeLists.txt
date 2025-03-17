# SPDX-FileCopyrightText: 2006-2025 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

project(handPointingR1Mk3)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

find_package(gb-ergocub-cartesian-service)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

if(NAVIGATION_USE_ROS2)
    find_package(YARP REQUIRED COMPONENTS sig dev os math)
else()
    find_package(YARP REQUIRED COMPONENTS sig dev os math)
endif()
include_directories(${ICUB_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${gb-ergocub-cartesian-service_LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Modules")
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
