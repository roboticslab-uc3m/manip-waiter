# Copyright: 2017 UC3M
# Author: Juan Miguel Garcia
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname kinematics)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)

yarp_install(FILES ${conf} DESTINATION ${WBPC-WAITER_CONTEXTS_INSTALL_DIR}/${appname})

