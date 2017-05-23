set(appname bashScripts)

file(GLOB bash_scripts ${CMAKE_CURRENT_SOURCE_DIR}/*.sh)

if (NOT WIN32)
yarp_install(FILES ${bash_scripts} DESTINATION $ENV{HOME})
endif()