/**
 * @ingroup zmpTEOwrist_programs
 * \defgroup zmpTEOwrist waiterExecManip
 *
 * @brief Creates an instance of roboticslab::zmpTEOwrist.
 *
 * @section zmpTEOwrist_legal Legal
 *
 * Copyright: 2017 (C) Universidad Carlos III de Madrid
 *
 * Author:Juan Miguel Garcia y Loli Pinel 2017
 *
 * CopyPolicy: This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 3.0 or later
 */

#include "MyRateThread.hpp"

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
        jr3Thread.port2.open("/waiter/jr3/ch2:i");  // Opening port associated to jr3 channel 2 (RIGHT ARM)
        jr3Thread.port3.open("/waiter/jr3/ch3:i");  // Opening port associated to jr3 channel 3 (LEFT ARM)
        jr3Thread.port0.open("/waiter/inertial:i"); // Opening port associated to inertial channel

    /** Connecting I/O YARP ports**/
        yarp::os::Time::delay(0.5);
        yarp.connect("/jr3/ch2:o","/waiter/jr3/ch2:i");
        if (jr3Thread.port2.getInputCount() == 0){
            cerr << "[error] Couldn't connect to YARP port /jr3/ch2." << endl;
        } else cout << "[success] Connected to RIGHT ARM JR3." << endl;
        yarp::os::Time::delay(0.5);

        yarp.connect("/jr3/ch3:o","/waiter/jr3/ch3:i");
        if (jr3Thread.port3.getInputCount() == 0){
            cerr << "[error] Couldn't connect to YARP port /jr3/ch3." << endl;
        } else cout << "[success] Connected to LEFT ARM JR3." << endl;
        yarp::os::Time::delay(0.5);

        yarp.connect("/inertial:o", "/waiter/inertial:i");
        if (jr3Thread.port0.getInputCount() == 0){
            cerr << "[error] Couldn't connect to YARP port /inertial." << endl;
        } else cout << "[success] Connected to IMU." << endl;
        yarp::os::Time::delay(0.5);


    jr3Thread.start();

    char c;
    do {
        c=getchar();
    } while (c != '\n');

    jr3Thread.stop();
    jr3Thread.port2.close();
    jr3Thread.port3.close();
    jr3Thread.port0.close();

    return 0;
}
