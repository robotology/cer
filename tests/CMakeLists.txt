# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

cmake_minimum_required(VERSION 2.8.9)
project(cer-tests)

## Tests using RTF - Robot Testing Framework

# find a version of RTF higher then 1.0.1
# and with yarp support enabled
find_package(RTF 1.1)

# set the output plugin directory to collect all the shared libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_SHARED_MODULE_PREFIX "")

if(RTF_FOUND)
   add_subdirectory(tripodTest)
   add_subdirectory(displayTest)
endif()

add_subdirectory(rgbdCalibrationTest)

