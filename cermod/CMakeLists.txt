include(YarpPlugin)

#########################################
# Prepare plugin library cermod
#########################################

yarp_begin_plugin_library(cermod)
  add_subdirectory(tripod)
  add_subdirectory(faceDisplayClient)
  add_subdirectory(faceDisplayServer)
yarp_end_plugin_library(cermod)
