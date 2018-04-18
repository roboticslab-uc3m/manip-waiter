/**
 * @ingroup testCap10_programs
 * \defgroup testCap10 waiterManip
 *
 * @brief Creates an instance of roboticslab::testCap10.
 *
 * @section testCap10_legal Legal
 *
 * Copyright: 2017 (C) Universidad Carlos III de Madrid
 *
 * Author:Juan Miguel Garcia 2018
 *
 * CopyPolicy: This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 3.0 or later
 */

#include "MyRateThread.hpp"

#include <math.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <fstream>
#include <deque>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <cmath>

#include "ICartesianControl.h"

int main(void) {

    /** Check yarp network**/
    yarp::os::Network yarp;

    printf("Checking network...\n");
    if (!yarp.checkNetwork()) {
        printf("Please start a yarp name server first\n");
       return(-1);
    }
    printf("Network ok\n");

    MyRateThread jr3Thread;
    /** Opening YARP ports**/
        //jr3Thread.port2.open("/waiter/jr3/ch2:i");  // Opening port associated to jr3 channel 2 (RIGHT ARM) // si fuera necesario
        jr3Thread.port3.open("/waiter/jr3/ch3:i");  // Opening port associated to jr3 channel 3 (LEFT ARM)
        jr3Thread.IMU.open("/waiter/inertial:i"); // Opening port associated to inertial channel

    /** Connecting I/O YARP ports**/
        yarp::os::Time::delay(0.5);
        // si fuera necesario
        /*yarp.connect("/jr3/ch2:o","/waiter/jr3/ch2:i");
        if (jr3Thread.port2.getInputCount() == 0){
            cerr << "[error] Couldn't connect to YARP port /jr3/ch2." << endl;
        } else cout << "[success] Connected to RIGHT ARM JR3." << endl;
        yarp::os::Time::delay(0.5);*/

        yarp.connect("/jr3/ch3:o","/waiter/jr3/ch3:i");
        if (jr3Thread.port3.getInputCount() == 0){
            cerr << "[error] Couldn't connect to YARP port /jr3/ch3." << endl;
        } else cout << "[success] Connected to LEFT ARM JR3." << endl;
        yarp::os::Time::delay(0.5);

        yarp.connect("/inertial:o", "/waiter/inertial:i");
        if (jr3Thread.IMU.getInputCount() == 0){
            cerr << "[error] Couldn't connect to YARP port /inertial." << endl;
        } else cout << "[success] Connected to IMU." << endl;
        yarp::os::Time::delay(0.5);


    /** Configuring LEFT ARM **/
    yarp::dev::IControlMode2 *leftArmIControlMode2;
    yarp::dev::IPositionControl2 *leftArmIPositionControl2;
    yarp::dev::IEncoders *leftArmIEncoders;
    yarp::os::Property leftArmOptions;
    leftArmOptions.put("device","remote_controlboard");
    leftArmOptions.put("remote","/teo/leftArm");
    leftArmOptions.put("local","/waiter/teo/leftArm");
    yarp::dev::PolyDriver leftArmDevice;
    leftArmDevice.open(leftArmOptions);
    if(!leftArmDevice.isValid()) {
        printf("robot leftArm device not available.\n");
        leftArmDevice.close();
        yarp::os::Network::fini();
        return false;
    }

    if (!leftArmDevice.view(leftArmIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring leftArmPos interface\n");
        return false;
    } else printf("[success] Acquired leftArmPos interface\n");

    if (!leftArmDevice.view(leftArmIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring leftArmIControlMode2 interface\n");
        return false;
    } else printf("[success] Acquired leftArmIControlMode2 interface\n");

    if (!leftArmDevice.view(leftArmIEncoders) ) {
        printf("[warning] Problems acquiring iEncoders interface\n");
        return false;
    } else printf("[success] Acquired iEncoders interface\n");

    /** Set control modes **/
    int leftArmAxes;
    leftArmIPositionControl2->getAxes(&leftArmAxes);
    std::vector<int> leftArmControlModes(leftArmAxes,VOCAB_CM_POSITION);
    if(! leftArmIControlMode2->setControlModes( leftArmControlModes.data() )){
        printf("[warning] Problems setting position control mode of: left-arm\n");
        return false;
    }
    /** Conection between zmpTEOwrist & jr3Thread **/
    jr3Thread.setIEncodersControl(leftArmIEncoders);
    jr3Thread.setIPositionControl2(leftArmIPositionControl2);
    //jr3Thread.setIVelocityControl2(leftArmIVelocityControl2);


    /** Solver device */
    roboticslab::ICartesianControl *iCartesianControl;

    //ICartesianSolver *iCartesianSolver;
    yarp::os::Property solverOptions;
    //solverOptions.fromString( rf.toString() );
    std::string solverStr = "KdlSolver";
    solverOptions.put("device",solverStr);
    solverOptions.put("cartesianRemote", "/teo/leftArm/CartesianControl"); // remote port through which we'll talk to the server
    solverOptions.put("cartesianLocal", "/CartesianControlExample");
    yarp::dev::PolyDriver solverDevice;
    solverDevice.open(solverOptions);

    if( ! solverDevice.isValid() )    {
        //CD_ERROR("solver device not valid: %s.\n",solverStr.c_str());
        printf("[ERROR] Solver device not valid: %s.\n",solverStr.c_str());
        return false;    }
    if( ! solverDevice.view(iCartesianControl) )    {
        //CD_ERROR("Could not view iCartesianSolver in: %s.\n",solverStr.c_str());
        printf("[ERROR] Could not view iCartesianSolver in: %s.\n",solverStr.c_str());
        return false;    }
    //jr3Thread.setICartesianSolver(iCartesianSolver);

    jr3Thread.start();

    char c;
    do {
        c=getchar();
    } while (c != '\n');

    jr3Thread.stop();
    //jr3Thread.port2.close(); // si fuera necesario
    jr3Thread.port3.close();
    jr3Thread.IMU.close();

    return 0;
}
