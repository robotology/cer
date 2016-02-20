# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

if(IPOPT_FOUND)
    add_subdirectory(reaching)
endif()

add_subdirectory(teleop)
add_subdirectory(baseControl)

