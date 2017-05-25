# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

project(cer-tests)

## Tests using RTF - Robot Testing Framework

# find a version of RTF higher then 1.0.1
# and with yarp support enabled

# set the output plugin directory to collect all the shared libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_SHARED_MODULE_PREFIX "")

find_package(RTF 1.1 COMPONENTS DLL)

if(RTF_FOUND)
    add_subdirectory(displayTest)
    if(IPOPT_FOUND)
        add_subdirectory(tripodTest)
    endif()
endif()

add_subdirectory(rgbdCalibrationTest)

