# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

project(cer_mobile_kinematics)

set(headers_private include/${PROJECT_NAME}/private/mobile_arm_common.h
                    include/${PROJECT_NAME}/private/mobile_arm_full_notorso_noheave.h)
set(headers         include/${PROJECT_NAME}/mobile_arm.h)
set(sources         src/mobile_arm.cpp)

source_group("Header Files" FILES ${headers_private} ${headers})
source_group("Source Files" FILES ${sources})

add_library(${PROJECT_NAME} ${headers_private} ${headers} ${sources})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${IPOPT_DEFINITIONS} _USE_MATH_DEFINES)

target_include_directories(${PROJECT_NAME} PUBLIC  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                                                   "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>"
                                           PRIVATE ${IPOPT_INCLUDE_DIRS})

set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
target_link_libraries(${PROJECT_NAME} cer_kinematics ctrlLib iKin ${YARP_LIBRARIES} ${IPOPT_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        EXPORT  ${VARS_PREFIX}
        COMPONENT runtime
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                     COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                     COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                     COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${PROJECT_NAME})

