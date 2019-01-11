# Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Scalzo
# email:  alessandro.scalzo@iit.it

project(R1ControlLib)

file(GLOB sources src/*.cpp)
file(GLOB headers include/*.h)

source_group("Header Files" FILES ${headers})
source_group("Source Files" FILES ${sources})

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/../R1ModelLib/include)
include_directories(${PROJECT_SOURCE_DIR}/../RobotModelLib/include)
include_directories(${PROJECT_SOURCE_DIR}/../RobotControlLib/include)

add_library(${PROJECT_NAME} ${headers} ${sources})
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME} RobotModelLib R1ModelLib RobotControlLib ${YARP_LIBRARIES})

# Add install target
yarp_install(TARGETS ${PROJECT_NAME}
             COMPONENT Runtime
             LIBRARY DESTINATION  ${CMAKE_INSTALL_LIBDIR}
             ARCHIVE DESTINATION  ${CMAKE_INSTALL_LIBDIR})

set_property(TARGET ${PROJECT_NAME}
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include)
