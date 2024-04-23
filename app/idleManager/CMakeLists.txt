set(appname idleManager)

file(GLOB conf      ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB apps      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB robotInterface        ${CMAKE_CURRENT_SOURCE_DIR}/conf/robotInterface/*.xml)

yarp_install(FILES ${conf}    DESTINATION ${CER_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${apps}    DESTINATION ${CER_APPLICATIONS_INSTALL_DIR})
yarp_install(FILES ${robotInterface}    DESTINATION ${CER_CONTEXTS_INSTALL_DIR}/${appname}/robotInterface)

