# SPDX-FileCopyrightText: 2006-2025 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

project(handPointingR1Mk3)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

find_package(ergocub-cartesian-control REQUIRED COMPONENTS 
                            gb-ergocub-cartesian-service)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

find_package(YARP REQUIRED COMPONENTS sig dev os math)

include_directories(${ICUB_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES}
                        ergocub-cartesian-control::gb-ergocub-cartesian-service
                        rclcpp::rclcpp
                        tf2::tf2
                        tf2_ros::tf2_ros
                        ${geometry_msgs_LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Modules")
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
