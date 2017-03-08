# Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Scalzo
# email:  alessandro.scalzo@iit.it

project(RobotModelLib)

file(GLOB sources src/*.cpp)
file(GLOB headers include/*.h)

source_group("Header Files" FILES ${headers})
source_group("Source Files" FILES ${sources})

include_directories(${PROJECT_SOURCE_DIR}/include)

add_definitions(-D_USE_MATH_DEFINES)
add_library(${PROJECT_NAME} ${headers} ${sources})
target_link_libraries(${PROJECT_NAME})

# Add install target
yarp_install(TARGETS ${PROJECT_NAME}
             COMPONENT Runtime
             LIBRARY DESTINATION  ${CMAKE_INSTALL_LIBDIR}
             ARCHIVE DESTINATION  ${CMAKE_INSTALL_LIBDIR})

set_property(TARGET ${PROJECT_NAME}
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include)
