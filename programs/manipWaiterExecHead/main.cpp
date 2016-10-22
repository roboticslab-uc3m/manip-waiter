// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup teo_body_programs
 * \defgroup waiterExecHead waiterExecHead
 *
 * @brief Creates an instance of teo::waiterExecHead.
 *
 * @section waiterExecHead_legal Legal
 *
 * Copyright: 2015 (C) Universidad Carlos III de Madrid
 *
 * Author: Juan Miguel Garcia
 *
 * CopyPolicy: This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 3.0 or later
 *
 * <hr>
 *
 * This file can be edited at waiterExecHead
 *
 */

#include <yarp/os/all.h>

#include "WaiterExecHead.hpp"

using namespace yarp::os;

int main(int argc, char **argv) {

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("waiterExecHead");
    rf.setDefaultConfigFile("waiterExecHead.ini");
    rf.configure(argc, argv);

    teo::WaiterExecHead mod;
    if(rf.check("help")) {
        return mod.runModule(rf);
    }

    printf("Run \"%s --help\" for options.\n",argv[0]);
    printf("%s checking for yarp network... ",argv[0]);
    fflush(stdout);
    Network yarp;
    if (!yarp.checkNetwork()) {
        fprintf(stderr,"[fail]\n%s found no yarp network (try running \"yarpserver &\"), bye!\n",argv[0]);
        return 1;
    } else printf("[ok]\n");

    return mod.runModule(rf);
}

