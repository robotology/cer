# Copyright: (C) 2015 iCub Facility
# Authors: Alberto Cardellino <alberto.cardellino@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(cerDoubleLidar
                    CATEGORY device
                    TYPE cer::dev::cerDoubleLidar
                    INCLUDE cerDoubleLidar.h
                    DEPENDS "NOT APPLE")


if(ENABLE_cermod_cerDoubleLidar)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    yarp_add_plugin(cerDoubleLidar cerDoubleLidar.h cerDoubleLidar.cpp)
    add_definitions(-D_USE_MATH_DEFINES)

    yarp_install(TARGETS cerDoubleLidar
                 EXPORT YARP_${YARP_PLUGIN_MASTER}
                 COMPONENT Runtime
                 LIBRARY DESTINATION ${CER_DYNAMIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${CER_PLUGIN_MANIFESTS_INSTALL_DIR})

    target_link_libraries(cerDoubleLidar ${YARP_LIBRARIES} )


endif(ENABLE_cermod_cerDoubleLidar)
