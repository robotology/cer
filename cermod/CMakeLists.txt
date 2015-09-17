
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpPlugin)
set(BUILD_SHARED_LIBS TRUE)

#########################################
# Prepare plugin library cermod
#########################################
yarp_configure_external_installation(cer WITH_PLUGINS)

message("CER_DYNAMIC_PLUGINS_INSTALL_DIR  is ${CER_DYNAMIC_PLUGINS_INSTALL_DIR}")
message("CER_PLUGIN_MANIFESTS_INSTALL_DIR is ${CER_PLUGIN_MANIFESTS_INSTALL_DIR}")

yarp_begin_plugin_library(cermod)
  add_subdirectory(tripod)
yarp_end_plugin_library(cermod)

yarp_add_plugin_yarpdev_executable(cermoddev cermod)
target_link_libraries(cermoddev cermod ${YARP_LIBRARIES})

install(TARGETS cermoddev DESTINATION bin)
install(TARGETS cermod EXPORT cermod
		LIBRARY DESTINATION lib COMPONENT shlib
		ARCHIVE DESTINATION lib
		PUBLIC_HEADER DESTINATION include/cermod COMPONENT dev)
