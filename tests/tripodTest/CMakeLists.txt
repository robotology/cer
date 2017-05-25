
#  Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
#  Authors: Alberto Cardellino <alberto.cardellino@iit.it>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

cmake_minimum_required(VERSION 2.8.9)

# set the project name
set(TESTNAME tripodTest)

# add the required cmake packages
find_package(YARP REQUIRED)

# add include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${YARP_HELPERS_INCLUDE_DIR})

# add the source codes to build the plugin library
add_library(${TESTNAME} MODULE tripodTest.h tripodTest.cpp)

# add required libraries
target_link_libraries(${TESTNAME} ${RTF_LIBRARIES} ${YARP_LIBRARIES} YARP::YARP_rtf)

# set the installation options
install(TARGETS ${TESTNAME}
        EXPORT ${TESTNAME}
        COMPONENT runtime
        LIBRARY DESTINATION lib)

