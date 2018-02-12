// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef _ratethread_H_
#define _ratethread_H_

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
#include <fstream>
#include <deque>

#include "ICartesianSolver.h"

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#define TS 0.03
using namespace std;

static FILE *fp;


/**
 * @ingroup zmpTEOwrist
 *
 * @brief ZMP_tray computation from left arm FT sensor
 *
 */
class MyRateThread : public yarp::os::RateThread
{
public:
    MyRateThread():RateThread(TS*1000.0){
        _d = 0.025;
    }
    void run();
    void ReadSensor();
    void AxesTransform();
    void ZMPcomp();

    yarp::os::Port port2;
    yarp::os::Port port3;
    yarp::os::Port IMU;

    void setIEncodersControl(yarp::dev::IEncoders *iEncoders) {
        this->leftArmIEncoders = iEncoders;        }
    void setIPositionControl2(yarp::dev::IPositionControl2 *iPositionControl2) {
        this->leftArmIPositionControl2 = iPositionControl2;        }
    //void setICartesianSolver(roboticslab::ICartesianSolver *iCartesianSolver) {
    //    this->iCartesianSolver = iCartesianSolver;        }

private:

    struct SensorDataJR3 {
        double fx, fy, fz;
        double mx, my, mz;
        yarp::os::Bottle bottle;
    } _sensor2, _sensor3;

    struct SensorDataIMU {
        double ang_x, ang_y, ang_z; // Angle (x,y,z) [deg]
        double acc_x, acc_y, acc_z; // Linear acceleration (x,y,z) [m/s^2]
        double spd_x, spd_y, spd_z; // Angular velocity (x,y,z) [deg/s]
        double mag_x, mag_y, mag_z; // Magnetic field (x,y,z)
        yarp::os::Bottle bottle;
    } _imu;

    deque<double> x_sensor, y_sensor, z_sensor; // to filter the Linear acceleration

    struct TrayData {
        double fx, fy, fz;
        double mx, my, mz;
        double xzmp, yzmp;
    } _tray;

    float _d;  //distance in mm between the plate center and the sensor center in the X axis

    //-- Robot device
    yarp::dev::IPositionControl2 *leftArmIPositionControl2;
    yarp::dev::IEncoders *leftArmIEncoders;

    //-- Solver device
    //roboticslab::ICartesianSolver *iCartesianSolver;

};

#endif //_ratethread_H_
