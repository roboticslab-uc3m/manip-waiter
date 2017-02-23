// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup jr3WristControl_programs
 * \defgroup jr3WristControl jr3WristControl
 *
 * @brief Creates an instance of teo::Jr3WristControl.
 *
 * @section jr3WristControl_legal Legal
 *
 * Copyright: 2016 (C) Universidad Carlos III de Madrid
 *
 * Author:Juan Miguel Garcia 2016
 *
 * CopyPolicy: This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 3.0 or later
 *
 * <hr>
 *
 * This file can be edited at waiterExecManip
 *
 */

#include <yarp/os/all.h>

#include "Jr3WristControl.hpp"


using namespace yarp::os;
int a=0;
int b=0;
int iteration=1;
float degrees;

int main(int argc, char **argv) {

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("jr3WristControl");
    rf.setDefaultConfigFile("jr3WristControl.ini");
    rf.configure(argc, argv);

    teo::WaiterExecManip mod;
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

