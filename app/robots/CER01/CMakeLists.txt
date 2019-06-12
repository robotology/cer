set(appname CER01)

set(scripts CER.xml yarprobotinterface.ini general.xml yarpmotorgui.ini)

yarp_install(FILES ${scripts} DESTINATION ${CER_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY calibrators DESTINATION ${CER_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY wrappers DESTINATION ${CER_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY hardware DESTINATION ${CER_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY sensors  DESTINATION ${CER_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY camera   DESTINATION ${CER_ROBOTS_INSTALL_DIR}/${appname})

