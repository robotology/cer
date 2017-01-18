#
# Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alberto Cardellino <alberto.cardellino@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(appname faceExpressionImage)


file(GLOB conf      ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB images    ${CMAKE_CURRENT_SOURCE_DIR}/images/*)
file(GLOB templates ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

yarp_install(FILES ${conf}      DESTINATION ${CER_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${images}    DESTINATION ${CER_CONTEXTS_INSTALL_DIR}/${appname}/images)
yarp_install(FILES ${templates} DESTINATION ${CER_APPLICATIONS_TEMPLATES_INSTALL_DIR})
