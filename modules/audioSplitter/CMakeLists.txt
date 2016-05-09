#
# Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Author: Francesco Diotalevi francesco.diotalevi@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#


set(PROJECTNAME audioSplitter)
project(${PROJECTNAME})

set(folder_source audioSplitter.cpp)
# file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
# source_group("Header Files" FILES ${folder_header})

include_directories( )

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

# target_link_libraries(${PROJECTNAME})

install(TARGETS ${PROJECTNAME} DESTINATION bin)

