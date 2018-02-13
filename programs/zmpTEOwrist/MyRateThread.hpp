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
#include <iomanip>
#include <fstream>
#include "ColorDebug.hpp"

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
        a = 0;
        b = 0;
        w = 0;
        inputAngle = -9;
        _d = 0.025; // distance in metres
        _l = 0.05;  // distance in metres
        _diff_time=0;

        quat.resize(4);
        quatC.resize(4);
        preFF.resize(4);
        FF.resize(4);

        _rzmp = 0;
        _rzmp_b = 0;
        x_zmp_b = 0;
        y_zmp_b = 0;

        _off._F.fx = 0;
        _off._F.fy = 0;
        _off._F.fz = 0;
        _off._M.mx = 0;
        _off._M.my = 0;
        _off._M.mz = 0;

        currentQ.resize(7);
        currentX.resize(7);
        desireX.resize(7);
        desireQ.resize(7);
        beforeQ.resize(7);

        _sensor2._initF.fx = 0;
        _sensor2._initF.fy = 0;
        _sensor2._initF.fz = 0;
        _sensor2._initT.mx = 0;
        _sensor2._initT.my = 0;
        _sensor2._initT.mz = 0;
    }
    bool threadInit();
    void run();
    void ReadSensor();
    void AxesTransform();
    void ZMPcomp();
    void saveToFile();

    //yarp::os::Port port2; // rightArm // si fuera necesario
    yarp::os::Port port3; // leftArm
    yarp::os::Port IMU;

    void setIEncodersControl(yarp::dev::IEncoders *iEncoders) {
        this->leftArmIEncoders = iEncoders;        }
    void setIPositionControl2(yarp::dev::IPositionControl2 *iPositionControl2) {
        this->leftArmIPositionControl2 = iPositionControl2;        }
    void setICartesianSolver(roboticslab::ICartesianSolver *iCartesianSolver) {
        this->iCartesianSolver = iCartesianSolver;        }

private:

    int a, b, w; // control porcess variables
    int numRobotJoints; // number of available joints
    int inputAngle; // angle input to move the trayTCP (openloop)
    int pepinito;   // no used on test1 (void mediumJR3(Bottle& FTsensor))

    float _d, _l;  // distance in m between the SDC tray and the SDC jr3

    double _rzmp, _rzmp_b, _rWorkSpace, _rFxy, _modFS, _modFF, x_zmp_b, y_zmp_b;
    double angle, _thetaX, _thetaY, _thetaXX, _thetaYY;

    std::vector<double> quat, quatC, preFF, FF, preFM, FM; // quaternios

    std::vector<double> currentQ, beforeQ, desireQ; // cartesian space variables
    std::vector<double> currentX, desireX;  // joint space variables

    double init_time, init_loop, curr_time, _diff_time, _dt;    // for calculating process time

    struct SensorDataJR3 { // lectura F/T del sensor JR3
        struct ForceVector {
            double fx, fy, fz;
        } _initF; // vector de fuerza proporcionado por el JR3
        struct TorqueVector {
            double mx, my, mz;
        } _initT; // vector de momento proporcionado por el JR3
        yarp::os::Bottle bottle;
    } _sensor2, _sensor3;
    struct SensorDataIMU { // imu data
        double ang_x, ang_y, ang_z; // Angle (x,y,z) [deg]
        double acc_x, acc_y, acc_z; // Linear acceleration (x,y,z) [m/s^2]
        double spd_x, spd_y, spd_z; // Angular velocity (x,y,z) [deg/s]
        double mag_x, mag_y, mag_z; // Magnetic field (x,y,z)
        yarp::os::Bottle bottle;
    } _imu;
    deque<double> x_sensor, y_sensor, z_sensor; // to filter the Linear acceleration
    struct TrayData { // variable para el control con los valores finales en la bandeja
        struct ForceVector {
            double fx, fy, fz;
        } _F; // vector de fuerza despues de la transformacion de orientacion
        struct TorqueVector {
            double mx, my, mz;
        } _M; // vector de momento despues de la transformacion de orientacion
        struct ZMPVector {
            double X, Y;
        } _zmp; // vector ZMP con sus dos componentes X e Y
        yarp::os::Bottle bottle;
    } _tray, _off;

    //-- Robot device
    yarp::dev::IPositionControl2 *leftArmIPositionControl2;
    yarp::dev::IEncoders *leftArmIEncoders;

    //-- Solver device
    roboticslab::ICartesianSolver *iCartesianSolver;

};

#endif //_ratethread_H_
