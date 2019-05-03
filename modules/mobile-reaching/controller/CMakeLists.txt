# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

project(cer_mobile-reaching-controller)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ctrlLib cer_mobile_kinematics)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

