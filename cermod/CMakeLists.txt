#########################################
# Prepare plugin library cermod
#########################################

yarp_begin_plugin_library(cermod QUIET)
if(IPOPT_FOUND)
  add_subdirectory(tripod)
endif(IPOPT_FOUND)

  add_subdirectory(faceDisplayClient)
  add_subdirectory(faceDisplayServer)
  add_subdirectory(oculus2cer)
  add_subdirectory(cer2headpose)
  add_subdirectory(cerDoubleLidar)
  add_subdirectory(r1face_mic)
yarp_end_plugin_library(cermod QUIET)
