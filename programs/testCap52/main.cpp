// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup TestCap52_programs
 * \defgroup TestCap52 TestCap52
 *
 * @brief Creates an instance of roboticslab::TestCap52.
 *
 * @section TestCap52 Legal
 *
 * Copyright: 2017 (C) Universidad Carlos III de Madrid
 *
 * Author:Juan Miguel Garcia 2018
 *
 * CopyPolicy: This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 3.0 or later
 *
 * <hr>
 *
 * This file can be edited at Test3
 *
 */

#include <yarp/os/all.h>

#include "TestCap52.hpp"


using namespace yarp::os;

//YARP_DECLARE_DEVICES(TestCap52)

int main(int argc, char **argv) {

    //YARP_REGISTER_DEVICES(Test3);

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("TestCap52");
    rf.setDefaultConfigFile("/usr/local/share/manip-waiter/contexts/kinematics/leftArmKinematics-waiter.ini");
    rf.configure(argc, argv);

    roboticslab::TestCap52 mod;
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
