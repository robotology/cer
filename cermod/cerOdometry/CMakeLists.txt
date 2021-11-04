#
# Copyright (C) 2021 IIT Istituto Italiano di Tecnologia
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
yarp_prepare_plugin(cerOdometry
        CATEGORY device
        TYPE CerOdometry
        INCLUDE CerOdometry.h
        DEPENDS "TARGET YARP::YARP_math"
        DEFAULT ON
        )

if(NOT SKIP_cerOdometry)
    yarp_add_plugin(cerOdometry)
#    include_directories(${ICUB_INCLUDE_DIRS})

    target_sources(cerOdometry
            PRIVATE
            CerOdometry.h
            CerOdometry.cpp
            )

    target_link_libraries(cerOdometry
            PRIVATE
            YARP::YARP_os
            YARP::YARP_sig
            YARP::YARP_dev
            YARP::YARP_math
            ctrlLib
            )
    list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS
            YARP_os
            YARP_sig
            YARP_dev
            YARP_math
            ctrlLib
            )

    yarp_install(
            TARGETS cerOdometry
            EXPORT YARP_${YARP_PLUGIN_MASTER}
            COMPONENT ${YARP_PLUGIN_MASTER}
            LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
            ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
            YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
    )

    set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

    set_property(TARGET cerOdometry PROPERTY FOLDER "Plugins/Device")
endif()
