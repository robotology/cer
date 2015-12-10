# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Scalzo
# email:  alessandro.scalzo@iit.it

project(cer_kinematics_alt)

set(headers_private include/${PROJECT_NAME}/private/Geometry.h
                    include/${PROJECT_NAME}/private/Matrix.h
                    include/${PROJECT_NAME}/private/Joints.h)
set(headers         include/${PROJECT_NAME}/Solver.h)
set(sources         src/Solver.cpp
                    src/Joints.cpp)

source_group("Header Files" FILES ${headers_private} ${headers})
source_group("Source Files" FILES ${sources})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${headers_private} ${headers} ${sources})

if(MSVC)
  target_compile_definitions(${PROJECT_NAME} PUBLIC _USE_MATH_DEFINES)
endif()

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})

# Add install target
yarp_install(TARGETS ${PROJECT_NAME}
             COMPONENT Runtime
             LIBRARY DESTINATION  ${CMAKE_INSTALL_LIBDIR}
             ARCHIVE DESTINATION  ${CMAKE_INSTALL_LIBDIR})

set_property(TARGET ${PROJECT_NAME}
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include
                                                          ${ICUB_INCLUDE_DIRS})

option(COMPILE_KIN_TESTS "Compile CER Kinematics Tests" OFF)
if(COMPILE_KIN_TESTS)
    add_subdirectory(tests)
endif()
