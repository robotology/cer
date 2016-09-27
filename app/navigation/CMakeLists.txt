
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install(FILES ${scripts} DESTINATION ${CER_APPLICATIONS_TEMPLATES_INSTALL_DIR})


