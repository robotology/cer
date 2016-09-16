# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

add_subdirectory(reaching)
add_subdirectory(gaze-controller)
add_subdirectory(teleop)
add_subdirectory(tripodJoystickControl)
add_subdirectory(robotJoystickControl)
add_subdirectory(tfPublisher)
add_subdirectory(textimage)

if(NOT WIN32)
    add_subdirectory(audioSplitter)
endif()
