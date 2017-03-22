# Copyright: (C) 2017 Istituto Italiano di Tecnologia
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU LGPL v2.0.

# Modules in YARP which needs to install ROS launch or config files should use this macro. This is used to set a common
# install prefix for multiple ROS erlated packages by mean of a environmental variable ROS_WS. If the variable is empty, 
# then the build / install folder for the current package will be used.
# A warning is issued to the user when the installation prefix in the CMake cache is different from the one chosen when 
# by the environmental variable.

macro(set_ros_config_install_prefix)
    if( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT         # This is an internal CMake variable
        OR ("${ROS_CONFIG_INSTALL_PREFIX}" STREQUAL "") )   # In case someone deleted the variable from the cache to refresh it
        
        # First thing: understand which build system has been used
        message(STATUS "\n<><><> Determine build system used <><><>")

        # default value, should not be necessary
        # set(BUILD_SYSTEM_USED "YARP_CMAKE" CACHE INTERNAL STRING)
        
        # When cmake is invoked, this variable will be set, when compiling with ament or plain cmake, it will not be set.
        if(DEFINED CATKIN_DEVEL_PREFIX)
            message(STATUS "CATKIN ON")
            # Calling catkin_package is required in order to generate installation prefix... this may cause problems
            # if this is a real catkin package and the macro is called twice, so check the variable catkin_found
            set(BUILD_SYSTEM_USED "CATKIN" CACHE INTERNAL STRING)
            if(NOT catkin_FOUND)
                find_package(catkin REQUIRED)
            endif()
            if("${CATKIN_PACKAGE_SHARE_DESTINATION}" STREQUAL "")
                catkin_package()
                message(WARNING "When using catkin, plase call 'catkin_package()' before calling 'set_ros_config_install_prefix()'")
            endif()

            set(ROS_CONFIG_INSTALL_PREFIX ${CATKIN_PACKAGE_SHARE_DESTINATION} CACHE PATH "ROS config & launch file install prefix" FORCE)
        
        elseif(${BUILT_WITH_AMENT_BUILD})  # This will (probably) be set by ament in the future, not implemented right now
            message(STATUS "AMENT ON")
            set(BUILD_SYSTEM_USED "AMENT" CACHE INTERNAL STRING)
            set(ROS_CONFIG_INSTALL_PREFIX share/${PROJECT_NAME} CACHE PATH "ROS config & launch file install prefix" FORCE)
        
        elseif("$ENV{ROS_WS}" STREQUAL "")     # Check if custom env var do exists  --> This may be replaced by a mechanism like iCub-Contrib-common ...
            message("Helper: no ROS_WS env var found")
                # if not, then we use standard yarp install dirs
                set(BUILD_SYSTEM_USED "YARP_CMAKE" CACHE INTERNAL STRING)

                string (TOUPPER ${PROJECT_NAME} UPNAME)  # Variables related to YARP data dirs have uppercase names
                set(ROS_CONFIG_INSTALL_PREFIX ${${UPNAME}_DATA_INSTALL_DIR} CACHE PATH "ROS config & launch file install prefix" FORCE)
        else()
                # if the var exists, use the var
                set(BUILD_SYSTEM_USED "YARP_CMAKE_WITH_ROS_INSTALL_DIR" CACHE INTERNAL STRING)
                message("Helper: ROS_WS is ${ROS_WS}")
                set(ROS_CONFIG_INSTALL_PREFIX  $ENV{ROS_WS}/${PROJECT_NAME} CACHE PATH "ROS config & launch file install prefix" FORCE)

        endif()
    endif()
endmacro()

##########################################################

include(CMakeParseArguments)
include(GNUInstallDirs)

# Export library function. Export a target to be used from external programs
#
# ROS_install_config_files( )
# Optional? force_ament force_catkin, force_yarp??
#
# This function installs the ROS config and launch files in the folders ROS expects them to be, along with the package.xml
# The folders config, launch and the package.xml have to be in the root of the package, as requested by ROS standard. 
# This installer works in 4 different ways, depending on the PC's set-up
# 
# 1) The 'package' (repo) is compiled and installed using plain CMake (the YARP way) and no env var is set.
#    In this case the ROS config files will be installed using YARP standard paths, i.e. by mean of  ${{PROJECT_NAME}_DATA_INSTALL_DIR}
#    In order to have ROS find this repo, the environment variable ROS_PACKAGE_PATH must be extended to include this folder.
#
# 2) The 'package' (repo) is compiled and installed using plain CMake (the YARP way) and the ROS_WS env var is set
#    This approach works like the iCub-Common package
#    In order to have ROS find this repo, the environment variable ROS_PACKAGE_PATH must be extended to include this folder.
#
# 3) The package is compiled using catkin ROS build system: this opens up 3 senarios:
#       - catkin_make: launch and config files are not copied/installed. Doing 'source devel/setup.*sh" the files in the src will be used.
#       - catkin_make install: launch and config files are installed in the install folder. Doing 'source devel/setup.*sh" the files in the 
#                              install folder will be used.
#       - catkin build: similar to catkin_make build. It installs by default
#
# 4) The package is compiled with ament build system. This is experimental and things in ROS2 may vary before official release.
#    As stated in this page https://github.com/ros2/ros2/wiki/Migration-Guide ROS2 will not use xml launch files anymore, but python scripts
#    instead. No ROS2 version of rosrun is ready yet. Right now we are installing launch and config files in ROS1 style for backward compatibility
#    so the package can be compiled with ament and run with rosrun/launch for ROS1 modules.
#

macro(ROS_config_install)  # extend it to list the folders to be installed?
# cmake_parse_arguments("VERBOSE")

#  if(VERBOSE)
    message(STATUS "*** Installing  ROS config files for project <${PROJECT_NAME}> into ${ROS_CONFIG_INSTALL_PREFIX}")
    message(STATUS "BUILD_SYSTEM_USED is ${BUILD_SYSTEM_USED}")
#  endif()


# Do stuff depending on build system


if("${BUILD_SYSTEM_USED}" STREQUAL "YARP_CMAKE")

    file(GLOB config    ${${PROJECT_NAME}_SOURCE_DIR}/config/*)
    file(GLOB launch    ${${PROJECT_NAME}_SOURCE_DIR}/launch/*)

    message("yarp install into ${ROS_CONFIG_INSTALL_PREFIX}")
    yarp_install(FILES ${config}    DESTINATION ${ROS_CONFIG_INSTALL_PREFIX}/config)
    yarp_install(FILES ${launch}    DESTINATION ${ROS_CONFIG_INSTALL_PREFIX}/launch)
    yarp_install(FILES package.xml  DESTINATION ${ROS_CONFIG_INSTALL_PREFIX})
endif()


if("${BUILD_SYSTEM_USED}" STREQUAL "YARP_CMAKE_WITH_ROS_INSTALL_DIR")

    file(GLOB config    ${${PROJECT_NAME}_SOURCE_DIR}/config/*)
    file(GLOB launch    ${${PROJECT_NAME}_SOURCE_DIR}/launch/*)
    
    # copy is done during cmake configuration, install does the copy during the make install
    file(COPY ${config}    DESTINATION ${ROS_CONFIG_INSTALL_PREFIX}/config)
    file(COPY ${launch}    DESTINATION ${ROS_CONFIG_INSTALL_PREFIX}/launch)
    file(COPY package.xml  DESTINATION ${ROS_CONFIG_INSTALL_PREFIX})
endif()

if("${BUILD_SYSTEM_USED}" STREQUAL "AMENT")  
    # Actually ROS2 / ament will not use xml launch files anymore, so I don't know if installing them 
    # is useful or not ... anyway it should not harm
    install(DIRECTORY ${${PROJECT_NAME}_SOURCE_DIR}/launch/
            DESTINATION ${ROS_CONFIG_INSTALL_PREFIX}/launch)
    
    install(DIRECTORY ${${PROJECT_NAME}_SOURCE_DIR}/config/
            DESTINATION ${ROS_CONFIG_INSTALL_PREFIX}/config)
    
    install(FILES package.xml  DESTINATION ${ROS_CONFIG_INSTALL_PREFIX})
endif()

if("${BUILD_SYSTEM_USED}" STREQUAL "CATKIN")
    install(DIRECTORY ${${PROJECT_NAME}_SOURCE_DIR}/launch/
            DESTINATION ${ROS_CONFIG_INSTALL_PREFIX}/launch)
    
    install(DIRECTORY ${${PROJECT_NAME}_SOURCE_DIR}/config/
            DESTINATION ${ROS_CONFIG_INSTALL_PREFIX}/config)
    
    install(FILES package.xml  DESTINATION ${ROS_CONFIG_INSTALL_PREFIX})
endif()

message("\n ------- Printing some stuff!! -------- \n")
string (TOUPPER ${PROJECT_NAME} UPNAME)
message("PROJECT_NAME_SOURCE_DIR                is ${${PROJECT_NAME}_SOURCE_DIR}")
message("${UPNAME}_DATA_INSTALL_DIR             is ${${UPNAME}_DATA_INSTALL_DIR}") 
message("CER_DATA_INSTALL_DIR                   is ${CER_DATA_INSTALL_DIR}")
message("CER_CONFIG_INSTALL_DIR                 is ${CER_CONFIG_INSTALL_DIR}")
message("CER_MODULES_INSTALL_DIR                is ${CER_MODULES_INSTALL_DIR}")
message("CER_APPLICATIONS_INSTALL_DIR           is ${CER_APPLICATIONS_INSTALL_DIR}")
message("CER_CONTEXTS_INSTALL_DIR               is ${CER_CONTEXTS_INSTALL_DIR}")
message("CER_APPLICATIONS_TEMPLATES_INSTALL_DIR is ${CER_APPLICATIONS_TEMPLATES_INSTALL_DIR}")

message("ROS_WS                                 is ${ROS_WS}")


endmacro(ROS_config_install)
