set(appname cerCluster)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.xml)
yarp_install(FILES ${conf} DESTINATION ${CER_CONTEXTS_INSTALL_DIR}/${appname})

set(icon ${CMAKE_CURRENT_SOURCE_DIR}/icon/icub-cluster-icon.png)
yarp_install(FILES ${icon} DESTINATION ${CER_CONTEXTS_INSTALL_DIR}/${appname})