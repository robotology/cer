set(appname handPointing)

file(GLOB conf      ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB templates ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB apps      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install(FILES ${conf}    DESTINATION ${CER_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${apps}    DESTINATION ${CER_APPLICATIONS_INSTALL_DIR})
yarp_install(FILES ${scripts} DESTINATION ${CER_APPLICATIONS_TEMPLATES_INSTALL_DIR})

