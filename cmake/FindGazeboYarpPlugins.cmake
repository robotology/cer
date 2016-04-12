#=============================================================================
# Copyright 2016  iCub Facility, Istituto Italiano di Tecnologia
#   Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of YCM, substitute the full
#  License text for the above reference.)

include(SelectLibraryConfigurations)
include(FindPackageHandleStandardArgs)


# Try to use CMake Config file to locate the package

# Disable package cache when not done automatically by CMake
# This is a workaround for CMake bug #14849
unset(_standard_find_module_registryArgs)
if(NOT CMAKE_REQUIRED_VERSION VERSION_LESS 3.1)
  message(AUTHOR_WARNING "Disabling cmake cache is supported since CMake 3.1. You can remove this check")
endif()
if(CMAKE_VERSION VERSION_LESS 3.1)
if(CMAKE_FIND_PACKAGE_NO_PACKAGE_REGISTRY)
  list(APPEND _standard_find_module_registryArgs NO_CMAKE_PACKAGE_REGISTRY)
endif()
if(CMAKE_FIND_PACKAGE_NO_SYSTEM_PACKAGE_REGISTRY)
  list(APPEND _standard_find_module_registryArgs NO_CMAKE_SYSTEM_PACKAGE_REGISTRY)
endif()
endif()

set(_GazeboYarpPlugins_FIND_QUIETLY ${GazeboYarpPlugins_FIND_QUIETLY})
find_package(GazeboYarpPlugins QUIET NO_MODULE ${_standard_find_module_registryArgs})
set(GazeboYarpPlugins_FIND_QUIETLY ${_GazeboYarpPlugins_FIND_QUIETLY})
unset(_GazeboYarpPlugins_FIND_QUIETLY)
mark_as_advanced(GazeboYarpPlugins_DIR)

if(GazeboYarpPlugins_FOUND)
  find_package_handle_standard_args(GazeboYarpPlugins
                                    FOUND_VAR GazeboYarpPlugins_FOUND
                                    REQUIRED_VARS GazeboYarpPlugins_CONFIG)
  return()
endif()


find_path(GazeboYarpPlugins_common_DIR
          NAMES GazeboYarpPlugins/common.h
          HINTS $ENV{GAZEBO_PLUGIN_ROOT}/libraries/common/include/
          DOC "Path to Gazebo YARP Plugins common headers")

find_path(GazeboYarpPlugins_singleton_DIR
          NAMES GazeboYarpPlugins/Handler.hh
          HINTS $ENV{GAZEBO_PLUGIN_ROOT}/libraries/singleton/include/
          DOC "Path to Gazebo YARP Plugins singleton headers")

mark_as_advanced(GazeboYarpPlugins_common_DIR
                 GazeboYarpPlugins_singleton_DIR)

set(GazeboYarpPlugins_INCLUDE_DIRS "${GazeboYarpPlugins_common_DIR}"
                                   "${GazeboYarpPlugins_singleton_DIR}")

list(REMOVE_DUPLICATES GazeboYarpPlugins_INCLUDE_DIRS)

find_library(GazeboYarpPlugins_singleton_LIBRARY_RELEASE
             NAMES gazebo_yarp_singleton
             DOC "Path to Gazebo YARP Plugins singleton library")
find_library(GazeboYarpPlugins_singleton_LIBRARY_DEBUG
             NAMES gazebo_yarp_singletond
             DOC "Path to Gazebo YARP Plugins singleton library (debug)")
select_library_configurations(GazeboYarpPlugins_singleton)

set(GazeboYarpPlugins_LIBRARIES ${GazeboYarpPlugins_singleton_LIBRARY})

find_package_handle_standard_args(GazeboYarpPlugins
                                  FOUND_VAR GazeboYarpPlugins_FOUND
                                  REQUIRED_VARS GazeboYarpPlugins_INCLUDE_DIRS
                                                GazeboYarpPlugins_LIBRARIES)


if(GazeboYarpPlugins_FOUND AND NOT TARGET GazeboYarpPlugins::singleton)
  add_library(GazeboYarpPlugins::singleton UNKNOWN IMPORTED)

  if(GazeboYarpPlugins_singleton_LIBRARY_RELEASE)
    set_property(TARGET GazeboYarpPlugins::singleton APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
    set_property(TARGET GazeboYarpPlugins::singleton        PROPERTY IMPORTED_LOCATION_RELEASE "${GazeboYarpPlugins_singleton_LIBRARY_RELEASE}" )
  endif()

  if(GazeboYarpPlugins_singleton_LIBRARY_DEBUG)
    set_property(TARGET GazeboYarpPlugins::singleton APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
    set_property(TARGET GazeboYarpPlugins::singleton        PROPERTY IMPORTED_LOCATION_DEBUG "${GazeboYarpPlugins_singleton_LIBRARY_DEBUG}" )
  endif()

  set_property(TARGET GazeboYarpPlugins::singleton APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${GazeboYarpPlugins_INCLUDE_DIRS}")
endif()
