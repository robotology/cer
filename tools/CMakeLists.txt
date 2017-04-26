# Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

if(IPOPT_FOUND)
  add_subdirectory(kinematics)
endif()

add_subdirectory(cer_log_joint)

