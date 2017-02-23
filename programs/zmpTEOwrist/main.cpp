/**
 * @ingroup zmpTEOwrist_programs
 * \defgroup zmpTEOwrist waiterExecManip
 *
 * @brief Creates an instance of teo::zmpTEOwrist.
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
    jr3Thread.port2.open("/jr3ch2:i"); //Opening port associated to jr3 channel 2 (RIGHT ARM)
    jr3Thread.port3.open("/jr3ch3:i"); //Opening port associated to jr3 channel 3 (LEFT ARM)

    /** Connecting I/O YARP ports**/
    yarp::os::Time::delay(0.5);
    yarp.connect("/jr3ch2:o","/jr3ch2:i");
    yarp::os::Time::delay(0.5);
    yarp.connect("/jr3ch3:o","/jr3ch3:i");

    jr3Thread.start();

    char c;
    do {
        c=getchar();
    } while (c != '\n');

    jr3Thread.stop();
    jr3Thread.port2.close();
    jr3Thread.port3.close();

    return 0;
}
