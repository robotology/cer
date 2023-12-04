# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

if(IPOPT_FOUND)
    add_subdirectory(reaching)
    add_subdirectory(gaze-controller)
    add_subdirectory(mobile-reaching)
endif()

add_subdirectory(altVelController)
add_subdirectory(selfCollision)

add_subdirectory(teleop)
add_subdirectory(tripodJoystickControl)
add_subdirectory(robotJoystickControl)
add_subdirectory(textimage)
add_subdirectory(faceExpression)
add_subdirectory(handPointing)
add_subdirectory(imagePatternRecognition)
add_subdirectory(idleManager)
add_subdirectory(gesticulate)
