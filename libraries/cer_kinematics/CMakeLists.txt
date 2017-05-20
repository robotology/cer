# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

project(cer_kinematics)

set(headers_private include/${PROJECT_NAME}/private/helpers.h
                    include/${PROJECT_NAME}/private/arm_common.h
                    include/${PROJECT_NAME}/private/arm_full_noheave.h
                    include/${PROJECT_NAME}/private/arm_full_heave.h
                    include/${PROJECT_NAME}/private/arm_full_notorso_noheave.h
                    include/${PROJECT_NAME}/private/arm_full_notorso_heave.h
                    include/${PROJECT_NAME}/private/arm_xyz_noheave.h
                    include/${PROJECT_NAME}/private/arm_xyz_heave.h
                    include/${PROJECT_NAME}/private/arm_xyz_notorso_noheave.h
                    include/${PROJECT_NAME}/private/arm_xyz_notorso_heave.h)
set(headers         include/${PROJECT_NAME}/utils.h
                    include/${PROJECT_NAME}/tripod.h
                    include/${PROJECT_NAME}/arm.h
                    include/${PROJECT_NAME}/head.h)
set(sources         src/helpers.cpp
                    src/utils.cpp
                    src/tripod.cpp
                    src/arm.cpp
                    src/head.cpp)

source_group("Header Files" FILES ${headers_private} ${headers})
source_group("Source Files" FILES ${sources})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${IPOPT_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

add_definitions(${IPOPT_DEFINITIONS} -D_USE_MATH_DEFINES)
add_library(${PROJECT_NAME} ${headers_private} ${headers} ${sources})

set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
target_link_libraries(${PROJECT_NAME} ctrlLib iKin ${YARP_LIBRARIES} ${IPOPT_LIBRARIES})

# Add install target
yarp_install(TARGETS ${PROJECT_NAME}
             COMPONENT Runtime
             LIBRARY DESTINATION  ${CMAKE_INSTALL_LIBDIR}
             ARCHIVE DESTINATION  ${CMAKE_INSTALL_LIBDIR})

set_property(TARGET ${PROJECT_NAME}
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include
                                                          ${ICUB_INCLUDE_DIRS})
