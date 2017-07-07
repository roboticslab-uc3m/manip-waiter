// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup Test2_programs
 * \defgroup Test2 Test2
 *
 * @brief Creates an instance of roboticslab::Test2.
 *
 * @section Test1 Legal
 *
 * Copyright: 2017 (C) Universidad Carlos III de Madrid
 *
 * Author:Juan Miguel Garcia 2017
 *
 * CopyPolicy: This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 3.0 or later
 *
 * <hr>
 *
 * This file can be edited at jr3WristControl
 *
 */

#include <yarp/os/all.h>

#include "Test2.hpp"


using namespace yarp::os;

//YARP_DECLARE_DEVICES(Test2)

int main(int argc, char **argv) {

    //YARP_REGISTER_DEVICES(Test2);

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("Test2");
    rf.setDefaultConfigFile("/usr/local/share/manip-waiter/contexts/kinematics/leftArmKinematics-waiter.ini");
    rf.configure(argc, argv);

    roboticslab::Test2 mod;
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
