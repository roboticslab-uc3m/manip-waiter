// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __IN_SR_PORT_HPP__
#define __IN_SR_PORT_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include <stdlib.h>
#include <fstream>
#include <stdio.h>
#include <math.h>  //-- fabs

#include "ICartesianSolver.h"

#include "ColorDebug.hpp"
#include <yarp/dev/IPositionDirect.h>

#define DEFAULT_QDOT_LIMIT 10

#define DEFAULT_STRATEGY "positionDirect"
//#define DEFAULT_STRATEGY "velocity"

static FILE *fp;

using namespace yarp::os;

namespace roboticslab
{

/**
 * @ingroup Jr3WristControl
 *
 * @brief Input port of Force/Torque data.
 *
 */
class InSrPort : public BufferedPort<Bottle> {
    public:

        InSrPort()        {
            a = 0;
            b = 0;
            w = 0;
            pepinito = -9;
            _d = 0.025;
            _l = 0.05;
            iteration=1;
            _diff_time=0;
            quat.resize(4);
            quatC.resize(4);
            preFF.resize(4);
            FF.resize(4);
            _rzmp = 0;
            _rzmp_b = 0;
            x_zmp_b = 0;
            y_zmp_b = 0;
            //preFM.resize(4);
            //FM.resize(4); // quaternios
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
            _jr3._initF.fx = 0;
            _jr3._initF.fy = 0;
            _jr3._initF.fz = 0;
            _jr3._initT.mx = 0;
            _jr3._initT.my = 0;
            _jr3._initT.mz = 0;



        }

        void setIEncodersControl(yarp::dev::IEncoders *iEncoders) {
            this->leftArmIEncoders = iEncoders;        }

        void setIPositionControl2(yarp::dev::IPositionControl2 *iPositionControl2) {
            this->leftArmIPositionControl2 = iPositionControl2;        }

        void setIVelocityControl2(yarp::dev::IVelocityControl2 *iVelocityControl2) {
            this->leftArmIVelocityControl2 = iVelocityControl2;        }

        void setICartesianSolver(roboticslab::ICartesianSolver *iCartesianSolver) {
            this->iCartesianSolver = iCartesianSolver;        }

    private:

        int a, b, w, numRobotJoints, pepinito, iteration;
        float _d, _l;  //distance in m between the SDC tray and the SDC jr3
        double initpos;

        struct SensorData { // lectura F/T del sensor JR3
            struct ForceVector {
                double fx, fy, fz;
            } _initF; // vector de fuerza proporcionado por el JR3
            struct TorqueVector {
                double mx, my, mz;
            } _initT; // vector de momento proporcionado por el JR3
        } _jr3, _med;

        struct TrayData { // variable para el control con los valores finales en la bandeja
            struct ForceVector {
                double fx, fy, fz;
            } _F; // vector de fuerza despues de la transformacion de orientacion
            struct TorqueVector {
                double mx, my, mz;
            } _M; // vector de momento despues de la transformacion de orientacion
            struct ZMPVector {
                double x_zmp, y_zmp;
            } _zmp; // vector ZMP con sus dos componentes X e Y
        } _tray, _off;

        double _rzmp, _rzmp_b, _rWorkSpace, _rFxy, _modFS, _modFF, x_zmp_b, y_zmp_b;
        double angle, _thetaX, _thetaY, _thetaXX, _thetaYY;
        std::vector<double> quat, quatC, preFF, FF, preFM, FM; // quaternios
        std::vector<double> currentQ, beforeQ, desireQ;
        std::vector<double> currentX, desireX;

        double init_time, init_loop, curr_time, _diff_time, _dt;


        //-- InSrPort Funtions
        void preprogrammedInitTrajectory();/** Set INITIAL POS-VEL-ACC **/
        virtual void onRead(Bottle& FTsensor);/** Callback on incoming Bottle. **/
        void ReadFTSensor(Bottle& FTsensor);/** Reading from the FT_JR3_sensor. **/
        void AxesTransform1();/** Rotation Transformation matrix of JR3. **/
        void AxesTransform2();/** Transformation matrix between JR3 and tray. **/
        void ZMPcomp();/** Calculating ZMP of the bottle. **/
        void LIPM3d();/** Control based on the 3D-LIMP. **/
        void saveToFile();/** Saving the ZMP measurements. **/
        void offSetJR3(Bottle& FTsensor);/** Offset JR3 measurements. **/
        void mediumJR3(Bottle& FTsensor);
        void getInitialTime();
        void getCurrentTime();

        //-- Robot device
        /** Left Arm Device */
        yarp::dev::PolyDriver leftArmDevice;
        /** Left Arm ControlMode2 Interface */
        yarp::dev::IControlMode2 *leftArmIControlMode2;
       /** Left Arm PositionControl2 Interface */
        yarp::dev::IPositionControl2 *leftArmIPositionControl2;
        /** Left Arm PositionControl2 Interface */
         yarp::dev::IVelocityControl2 *leftArmIVelocityControl2; // actualmente no se utilizado
        /** Left Arm Encoders Interface */
        yarp::dev::IEncoders *leftArmIEncoders;

        //-- Solver device
        roboticslab::ICartesianSolver *iCartesianSolver;

};

}  // namespace roboticslab

#endif // __IN_SR_PORT_HPP__
