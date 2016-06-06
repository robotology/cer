# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname tfPublisher)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
yarp_install(FILES ${conf} DESTINATION ${CER_CONTEXTS_INSTALL_DIR}/${appname})


