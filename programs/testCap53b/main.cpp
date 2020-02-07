// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup TestCap53b_programs
 * \defgroup TestCap53b TestCap53b
 *
 * @brief Creates an instance of roboticslab::WaiterExecManip.
 *
 * @section TestCap53b Legal
 *
 * Copyright: 2019 (C) Universidad Carlos III de Madrid
 *
 * Author:Juan Miguel Garcia 2020
 *
 * CopyPolicy: This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 3.0 or later
 *
 * <hr>
 *
 * This file can be edited at TestCap54
 *
 */

#include "TestCap53b.hpp"

int main(int argc, char **argv) {

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("TestCap53b");
    rf.configure(argc, argv);

    roboticslab::TestCap53b mod;
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
