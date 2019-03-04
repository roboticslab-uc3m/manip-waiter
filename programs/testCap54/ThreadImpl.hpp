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

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace roboticslab;

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

    void setIEncodersControl(IEncoders *iRightLegEncoders, IEncoders *iLeftLegEncoders) {
        this->rightLegIEncoders = iRightLegEncoders;
        this->leftLegIEncoders = iLeftLegEncoders;        }

    void setIPositionControl2(IPositionControl2 *iRightLegPositionControl2,IPositionControl2 *iLeftLegPositionControl2) {
        this->rightLegIPositionControl2 = iRightLegPositionControl2;
        this->leftLegIPositionControl2 = iLeftLegPositionControl2;        }

    void setIVelocityControl2(IVelocityControl2 *iRightLegVelocityControl2,IVelocityControl2 *iLeftLegVelocityControl2) {
        this->rightLegIVelocityControl2 = iRightLegVelocityControl2;
        this->leftLegIVelocityControl2 = iLeftLegVelocityControl2;        }

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

        LIPM2d _evalLIPM; // Discrete-time Space State DLIPM Model evaluation


        //-- IMU data port
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
        void evaluateModel(std::vector<double> &rightLegQs,std::vector<double> &leftLegQs);/** Calculating OUTPUT (Qi) of the legs. **/
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
        /** Head ControlMode2 Interface */
        yarp::dev::IControlMode2 *headIControlMode2;
        /** Head PositionControl2 Interface */
        yarp::dev::IPositionControl2 *headIPositionControl2; // para control en posicion
        /** Head VelocityControl2 Interface */
        yarp::dev::IVelocityControl2 *headIVelocityControl2; // para control en velocidad

        /** Axes number **/
        int numLeftArmJoints;
        /** Left Arm Device */
        yarp::dev::PolyDriver leftArmDevice;
        /** Encoders **/
        yarp::dev::IEncoders *leftArmIEncoders;
        /** Left Arm ControlMode2 Interface */
        yarp::dev::IControlMode2 *leftArmIControlMode2;
        /** Left Arm PositionControl2 Interface */
        yarp::dev::IPositionControl2 *leftArmIPositionControl2; // para control en posicion
        /** Left Arm VelocityControl2 Interface */
        yarp::dev::IVelocityControl2 *leftArmIVelocityControl2; // para control en velocidad
        /** FT 3 AnalogSensor Interface */
        yarp::dev::IAnalogSensor *ft3AnalogSensor;

        /** Axes number **/
        int numRightArmJoints;
        /** Right Arm Device */
        yarp::dev::PolyDriver rightArmDevice;
        /** Encoders **/
        yarp::dev::IEncoders *rightArmIEncoders;
        /** Right Arm ControlMode2 Interface */
        yarp::dev::IControlMode2 *rightArmIControlMode2;
        /** Right Arm PositionControl2 Interface */
        yarp::dev::IPositionControl2 *rightArmIPositionControl2; // para control en posicion
        /** Right Arm VelocityControl2 Interface */
        yarp::dev::IVelocityControl2 *rightArmIVelocityControl2; // para control en velocidad
        /** FT 2 AnalogSensor Interface */
        yarp::dev::IAnalogSensor *ft2AnalogSensor;

        //-- lower body devices variables
        /** Axes number **/
        int numTrunkJoints;
        /** Trunk Device */
        yarp::dev::PolyDriver trunkDevice;
        /** Encoders **/
        yarp::dev::IEncoders *trunkIEncoders;
        /** Trunk ControlMode2 Interface */
        yarp::dev::IControlMode2 *trunkIControlMode2;
        /** Trunk PositionControl2 Interface */
        yarp::dev::IPositionControl2 *trunkIPositionControl2; // para control en posicion
        /** Trunk VelocityControl2 Interface */
        yarp::dev::IVelocityControl2 *trunkIVelocityControl2; // para control en velocidad

        /** Axes number **/
        int numLeftLegJoints;
        /** Left Leg Device */
        yarp::dev::PolyDriver leftLegDevice;
        /** Encoders **/
        yarp::dev::IEncoders *leftLegIEncoders;
        /** Left Leg ControlMode2 Interface */
        yarp::dev::IControlMode2 *leftLegIControlMode2;
        /** Left Leg PositionControl2 Interface */
        yarp::dev::IPositionControl2 *leftLegIPositionControl2; // para control en posicion
        /** Left Leg VelocityControl2 Interface */
        yarp::dev::IVelocityControl2 *leftLegIVelocityControl2; // para control en velocidad
        /** FT 1 AnalogSensor Interface */
        yarp::dev::IAnalogSensor *ft1AnalogSensor;

        /** Axes number **/
        int numRightLegJoints;
        /** Right Leg Device */
        yarp::dev::PolyDriver rightLegDevice;
        /** Encoders **/
        yarp::dev::IEncoders *rightLegIEncoders;
        /** Right Leg ControlMode2 Interface */
        yarp::dev::IControlMode2 *rightLegIControlMode2;
        /** Right Leg PositionControl2 Interface */
        yarp::dev::IPositionControl2 *rightLegIPositionControl2; // para control en posicion
        /** Right Leg VelocityControl2 Interface */
        yarp::dev::IVelocityControl2 *rightLegIVelocityControl2; // para control en velocidad
        /** FT 0 AnalogSensor Interface */
        yarp::dev::IAnalogSensor *ft0AnalogSensor;

};

}  // namespace roboticslab

#endif // __THREAD_IMPL_HPP__
