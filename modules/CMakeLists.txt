# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

if(IPOPT_FOUND)
    add_subdirectory(reaching)
    add_subdirectory(gaze-controller)
endif()

add_subdirectory(teleop)
add_subdirectory(tripodJoystickControl)
add_subdirectory(robotJoystickControl)
add_subdirectory(tfPublisher)
add_subdirectory(textimage)
add_subdirectory(faceExpression)
#add_subdirectory(altVelController)
#add_subdirectory(selfCollision)

if(NOT WIN32)
    add_subdirectory(audioSplitter)
endif()
