// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup asibot_examples_cpp
 * \defgroup testRemoteRaveBot testRemoteRaveBot
 *
 * @brief This example connects to a running \ref testRaveBot or \ref cartesianServer module.
 *
 * <b>Legal</b>
 *
 * Copyright: (C) 2010 Universidad Carlos III de Madrid;
 *            (C) 2010 RobotCub Consortium
 *
 * Author: Juan Hernandez
 *
 * Contribs: Paul Fitzpatrick and Giacomo Spigler (YARP dev/motortest.cpp example)
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * <b>Building</b>
\verbatim
cd $ASIBOT_ROOT/example/cpp
mkdir build; cd build; cmake ..
make -j3
\endverbatim
 *
 * <b>Running</b>
\verbatim
./testRemoteRaveBot
\endverbatim
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include <fstream>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;
int a=0;
int b=0;
int iteration=1;

int main(int argc, char *argv[]) {

    time_t timev;

    printf("WARNING: requires a running instance of RaveBot (i.e. testRaveBot or cartesianServer)\n");
    Network yarp;
    if (!Network::checkNetwork()) {
        printf("Please start a yarp name server first\n");
        return(-1);
    }
    Property options;
    options.put("device","remote_controlboard");
    options.put("remote","/teo/leftArm");
    options.put("local","/local");
    PolyDriver dd(options);
    if(!dd.isValid()) {
      printf("RaveBot device not available.\n");
	  dd.close();
      Network::fini();
      return 1;
    }

    IPositionControl *pos;
    bool ok = dd.view(pos);
    if (!ok) {
        printf("[warning] Problems acquiring robot interface\n");
        return false;
    } else printf("[success] testAsibot acquired robot interface\n");
    pos->setPositionMode();

    for(b=0; b<20; b++){
        a=0;
        if (a==0) {
            printf("MOVE TO POSITION 01\n");
            pos->positionMove(0, -30);
            pos->positionMove(1, 40);
            pos->positionMove(2, 0);
            pos->positionMove(3, -70);
            pos->positionMove(4, -40);
            pos->positionMove(5, 10);
            pos->positionMove(6, 0);

            Time::delay(5);
            a=1;
        }
        
        if (a==1) {
            printf("MOVE TO POSITION 02\n");
            pos->positionMove(0, -10);
            pos->positionMove(1, 30);
            pos->positionMove(2, 0);
            pos->positionMove(3, -90);
            pos->positionMove(4, -30);
            pos->positionMove(5, 10);
            pos->positionMove(6, 0);

            Time::delay(5);
            a=2;
        }
        
        if (a==2) {
            printf("MOVE TO POSITION 03\n");
            pos->positionMove(0, -30);
            pos->positionMove(1, -10);
            pos->positionMove(2, 0);
            pos->positionMove(3, -70);
            pos->positionMove(4, 10);
            pos->positionMove(5, 10);
            pos->positionMove(6, 0);

            Time::delay(5);
            a=0;
        }
    }


  /*  b=5;
    a=0;
    if(a==0&&b>=0){
    printf("test positionMove(1,-35)\n");
    pos->positionMove(1, b);
    b--;
    } */

    printf("Delaying 5 seconds...\n");
    Time::delay(5);

 /*   IEncoders *enc;
    ok = dd.view(enc);

    IVelocityControl *vel;
    ok = dd.view(vel);
    vel->setVelocityMode();
    printf("test velocityMove(0,10)\n");
    vel->velocityMove(0,10);

    printf("Delaying 5 seconds...\n");
    Time::delay(5); */

    return 0;
}
