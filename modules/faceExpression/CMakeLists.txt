#
# Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alberto Cardellino <alberto.cardellino@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

project(faceExpressionImage)

option(ENABLE_faceExpressionImage "Compile blinking module for face display" false)

if(ENABLE_faceExpressionImage)

    ## Use opencv
    find_package(OpenCV)
    if(NOT OpenCV_FOUND)
        message(WARNING "OpenCV was not found therefore FaceExpression cannot be compiled. Please disable it or install OpenCV.")
        return()
    endif(NOT OpenCV_FOUND)


    file(GLOB folder_source main.cpp faceExpressionImage.cpp)
    file(GLOB folder_header faceExpressionImage.hpp)

    source_group("Source Files" FILES ${folder_source})
    source_group("Header Files" FILES ${folder_header})

    include_directories(${YARP_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIR})

    add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})

    target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
    link_directories(${OpenCV_LIB_DIR})


    install(TARGETS ${PROJECT_NAME} DESTINATION bin)

endif(ENABLE_faceExpressionImage)