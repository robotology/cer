set(appname CER01)

set(scripts CER_torso_only.xml yarprobotinterface.ini general.xml yarpmotorgui.ini)

yarp_install(FILES ${scripts} DESTINATION ${CER_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY calibrators DESTINATION ${CER_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY wrappers DESTINATION ${CER_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(DIRECTORY hardware DESTINATION ${CER_ROBOTS_INSTALL_DIR}/${appname})

