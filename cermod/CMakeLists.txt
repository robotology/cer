include(YarpPlugin)

#########################################
# Prepare plugin library cermod
#########################################

yarp_begin_plugin_library(cermod)
if(IPOPT_FOUND)
  add_subdirectory(tripod)
endif(IPOPT_FOUND)

  add_subdirectory(faceDisplayClient)
  add_subdirectory(faceDisplayServer)
  add_subdirectory(oculus2cer)
yarp_end_plugin_library(cermod)
