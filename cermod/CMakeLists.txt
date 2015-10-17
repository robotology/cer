
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpPlugin)
set(BUILD_SHARED_LIBS TRUE)

#########################################
# Prepare plugin library cermod
#########################################
yarp_configure_external_installation(cer WITH_PLUGINS)

yarp_begin_plugin_library(cermod)
  add_subdirectory(tripod)
yarp_end_plugin_library(cermod)

install(TARGETS cermod EXPORT cermod
		LIBRARY DESTINATION lib COMPONENT shlib
		ARCHIVE DESTINATION lib
		PUBLIC_HEADER DESTINATION include/cermod COMPONENT dev)
