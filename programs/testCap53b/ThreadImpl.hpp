// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __THREAD_IMPL_HPP__
#define __THREAD_IMPL_HPP__

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
#include "ColorDebug.hpp"
#include <iomanip>

#include "KinematicRepresentation.hpp"
#include "ICartesianSolver.h"

#include "LIPM2d.h"
#include "global.h"

#include "fcontrol.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace roboticslab;
using namespace KinRepresentation;

#define DEFAULT_QDOT_LIMIT 10
#define DEFAULT_RATE_MS 20

#define DEFAULT_STRATEGY "Position"
//#define DEFAULT_STRATEGY "positionDirect"
//#define DEFAULT_STRATEGY "velocity"

static FILE *fp;

namespace roboticslab
{

/**
 * @ingroup ThreadImpl
 *
 * @brief Input port of Force/Torque data.
 *
 */
class ThreadImpl : public yarp::os::Thread {
    public:

/*    void setNumJoints(int *iNumLeftArmJoints, int *iNumRightArmJoints) {
        this->numLeftArmJoints = iNumLeftArmJoints;
        this->numRightArmJoints = iNumRightArmJoints;        }*/

    void setIEncodersControl(IEncoders *iRightLegEncoders, IEncoders *iLeftLegEncoders) {
        this->rightLegIEncoders = iRightLegEncoders;
        this->leftLegIEncoders = iLeftLegEncoders;        }

    void setIPositionControl(IPositionControl *iRightLegPositionControl,IPositionControl *iLeftLegPositionControl) {
        this->rightLegIPositionControl = iRightLegPositionControl;
        this->leftLegIPositionControl = iLeftLegPositionControl;        }

    void setIPositionDirect(IPositionDirect *iRightLegPositionDirect,IPositionDirect *iLeftLegPositionDirect) {
        this->rightLegIPositionDirect = iRightLegPositionDirect;
        this->leftLegIPositionDirect = iLeftLegPositionDirect;        }

    void setIVelocityControl(IVelocityControl *iRightLegVelocityControl,IVelocityControl *iLeftLegVelocityControl) {
        this->rightLegIVelocityControl = iRightLegVelocityControl;
        this->leftLegIVelocityControl = iLeftLegVelocityControl;        }

    void setInputPorts(yarp::os::Port *inputPortImu) {
        this->portImu = inputPortImu;   }

    void setIAnalogSensor(IAnalogSensor *iFT0AnalogSensor,IAnalogSensor *iFT1AnalogSensor,IAnalogSensor *iFT2AnalogSensor,IAnalogSensor *iFT3AnalogSensor) {
        this->ft0AnalogSensor = iFT0AnalogSensor;
        this->ft1AnalogSensor = iFT1AnalogSensor;
        this->ft2AnalogSensor = iFT2AnalogSensor;
        this->ft3AnalogSensor = iFT3AnalogSensor;        }


protected:

        //-- IMU variables
        double acc_x, acc_y, acc_z;
        double ang_x, ang_y, ang_z;
        double spd_x, spd_y, spd_z;
        double mag_x, mag_y, mag_z;
        deque<double> x_sensor, y_sensor, z_sensor;
        //-- IMU LOW-FILTER variables & CONVERTION
        double ddx, ddy, ddz, ddx_robot, ddy_robot, ddz_robot;

        //-- struct variable for F/T reading from the JR3 sensors
        struct SensorFT_Data {
            struct ForceVector {
                double fx, fy, fz;
            } _F; // force vector by JR3
            struct TorqueVector {
                double mx, my, mz;
            } _T; // torque vector by JR3
        } _LL, _RL, _LA, _RA, _med;

        int a, b, n; // main process control variables

        //-- FT & IMU LOW-FILTER variables
        float offs_x_imu, offs_x_ft; // zmp offset in initial time - frontal plane
        float offs_y_imu, offs_y_ft; // zmp offset in initial time - frontal plane
        float sum_x_imu, sum_y_imu, sum_x_ft, sum_y_ft; // adding offset values.
        float test;

        //-- ZMP & IMU variables
        float _xzmp_ft0, _yzmp_ft0; // ZMP-FT sensor 0 (right)
        float _xzmp_ft1, _yzmp_ft1; // ZMP-FT sensor 1 (left)
        float _xzmp_ft01, _yzmp_ft01; // ZMP double support
        float Xzmp_ft, Yzmp_ft; // Global ZMP-FT after filter
        double Xzmp_imu, Yzmp_imu; // Global ZMP-IMU after filter

        //-- Control/Movement variables
        float zmp_ref, _ang_ref, _ang_out, ka;

        //-- Time variables
        double init_time, act_time, init_loop, act_loop, it_time, it_prev; // for calculating process time


        //-- iDENTIFICATION
        OnlineSystemIdentification id1;
        vector<double> *num, *den;

        LIPM2d _evalLIPM; // Discrete-time Space State DLIPM Model evaluation


        //-- Sensors variables
        yarp::os::Port *portFt0;
        yarp::os::Port *portFt1;
        yarp::os::Port *portFt2;
        yarp::os::Port *portFt3;
        yarp::os::Port *portImu;


        //-- ThreadImpl Funtions
        bool threadInit();
        void run();

        void readSensorsFT0();/** Reading from the FT0_JR3_sensor. **/
        void readSensorsFT1();/** Reading from the FT1_JR3_sensor. **/
        void readSensorsFT2();/** Reading from the FT2_JR3_sensor. **/
        void readSensorsFT3();/** Reading from the FT3_JR3_sensor. **/
        void readSensorsIMU();/** Reading from the IMU_XSENS_sensor. **/

        void zmpCompFT();/** Calculating ZMP-FT of the body. **/
        void zmpCompIMU();/** Calculating ZMP-IMU of the body. **/
        void evaluateModel();/** Calculating OUTPUT (Qi) of the legs based on D-LIPM. **/
        void evaluateLIPM();/** Calculating OUTPUT (Qi) of the legs based on LIPM. **/
        void setJoints();/** Position control. **/

        void printData();/** Printing data info on terminal **/
        void confCSVfile();/** Configuring CSV file **/
        void saveInFileCsv();/** Saving the ZMP measurements. **/

        void getInitialTime();
        void getCurrentTime();

        //-- upper body devices variables
        /** Axes number **/
        int numHeadJoints;
        /** Head Device */
        yarp::dev::PolyDriver headDevice;
        /** Encoders **/
        yarp::dev::IEncoders *headIEncoders;
        /** Head ControlMode Interface */
        yarp::dev::IControlMode *headIControlMode;
        /** Head PositionControl Interface */
        yarp::dev::IPositionControl *headIPositionControl; // para control en posicion
        /** Head VelocityControl Interface */
        yarp::dev::IVelocityControl *headIVelocityControl; // para control en velocidad

        /** Axes number **/
        int numLeftArmJoints;
        /** Left Arm Device */
        yarp::dev::PolyDriver leftArmDevice;
        /** Encoders **/
        yarp::dev::IEncoders *leftArmIEncoders;
        /** Left Arm ControlMode Interface */
        yarp::dev::IControlMode *leftArmIControlMode;
        /** Left Arm PositionControl Interface */
        yarp::dev::IPositionControl *leftArmIPositionControl; // para control en posicion
        /** Left Arm VelocityControl Interface */
        yarp::dev::IVelocityControl *leftArmIVelocityControl; // para control en velocidad
        /** FT 3 AnalogSensor Interface */
        yarp::dev::IAnalogSensor *ft3AnalogSensor;

        /** Axes number **/
        int numRightArmJoints;
        /** Right Arm Device */
        yarp::dev::PolyDriver rightArmDevice;
        /** Encoders **/
        yarp::dev::IEncoders *rightArmIEncoders;
        /** Right Arm ControlMode Interface */
        yarp::dev::IControlMode *rightArmIControlMode;
        /** Right Arm PositionControl Interface */
        yarp::dev::IPositionControl *rightArmIPositionControl; // para control en posicion
        /** Right Arm VelocityControl Interface */
        yarp::dev::IVelocityControl *rightArmIVelocityControl; // para control en velocidad
        /** FT 2 AnalogSensor Interface */
        yarp::dev::IAnalogSensor *ft2AnalogSensor;

        //-- lower body devices variables
        /** Axes number **/
        int numTrunkJoints;
        /** Trunk Device */
        yarp::dev::PolyDriver trunkDevice;
        /** Encoders **/
        yarp::dev::IEncoders *trunkIEncoders;
        /** Trunk ControlMode Interface */
        yarp::dev::IControlMode *trunkIControlMode;
        /** Trunk PositionControl Interface */
        yarp::dev::IPositionControl *trunkIPositionControl; // para control en posicion
        /** Trunk VelocityControl Interface */
        yarp::dev::IVelocityControl *trunkIVelocityControl; // para control en velocidad

        /** Axes number **/
        int numLeftLegJoints;
        /** Left Leg Device */
        yarp::dev::PolyDriver leftLegDevice;
        /** Encoders **/
        yarp::dev::IEncoders *leftLegIEncoders;
        /** Left Leg ControlMode Interface */
        yarp::dev::IControlMode *leftLegIControlMode;
        /** Left Leg PositionControl Interface */
        yarp::dev::IPositionControl *leftLegIPositionControl; // para control en posicion
        /** Left Leg PositionDirect Interface */
        yarp::dev::IPositionDirect *leftLegIPositionDirect; // para control en posicion
        /** Left Leg VelocityControl Interface */
        yarp::dev::IVelocityControl *leftLegIVelocityControl; // para control en velocidad
        /** FT 1 AnalogSensor Interface */
        yarp::dev::IAnalogSensor *ft1AnalogSensor;

        /** Axes number **/
        int numRightLegJoints;
        /** Right Leg Device */
        yarp::dev::PolyDriver rightLegDevice;
        /** Encoders **/
        yarp::dev::IEncoders *rightLegIEncoders;
        /** Right Leg ControlMode Interface */
        yarp::dev::IControlMode *rightLegIControlMode;
        /** Right Leg PositionControl Interface */
        yarp::dev::IPositionControl *rightLegIPositionControl; // para control en posicion
        /** Right Leg PositionDirect Interface */
        yarp::dev::IPositionDirect *rightLegIPositionDirect; // para control en posicion
        /** Right Leg VelocityControl Interface */
        yarp::dev::IVelocityControl *rightLegIVelocityControl; // para control en velocidad
        /** FT 0 AnalogSensor Interface */
        yarp::dev::IAnalogSensor *ft0AnalogSensor;
};

}  // namespace roboticslab

#endif // __THREAD_IMPL_HPP__
