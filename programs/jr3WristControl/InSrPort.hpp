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


using namespace yarp::os;

namespace teo
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
            follow = 0;
            a = 0;
            b = 0;
            pepinito = 1;
            _d = 0.025;
            _l = 0.05;
            iteration=1;
            quat.resize(4);
            quatC.resize(4);
            preFF.resize(4);
            FF.resize(4);
            preFM.resize(4);
            FM.resize(4); // quaternios
            _off._F.fx = 0;
            _off._F.fy = 0;
            _off._F.fz = 0; // No interesa eliminar
            _off._M.mx = 0;
            _off._M.my = 0;
            _off._M.mz = 0;
            currentQ.resize(7);

        }

        void setIEncodersControl(yarp::dev::IEncoders *iEncoders) {
            this->iEncoders = iEncoders;        }
        void setIPositionControl(yarp::dev::IPositionControl *iPositionControl) {
            this->iPositionControl = iPositionControl;        }
        void setIPositionDirect(yarp::dev::IPositionDirect *iPositionDirect) {
            this->iPositionDirect = iPositionDirect;        }
        void setIVelocityControl(yarp::dev::IVelocityControl *iVelocityControl) {
            this->iVelocityControl = iVelocityControl;        }
        void setICartesianSolver(teo::ICartesianSolver *iCartesianSolver) {
            this->iCartesianSolver = iCartesianSolver;        }
        void setFollow(int value);

        //yarp::os::Port port2; posibilidad de usar la muñeca derecha
        yarp::os::Port port3;

    private:

        int follow, a, b, numRobotJoints, pepinito, iteration;
        float _d, _l;  //distance in m between the SDC tray and the SDC jr3
        double initpos;

        struct SensorData { // lectura F/T del sensor JR3
            struct ForceVector {
                double fx, fy, fz;
            } _initF; // vector de fuerza proporcionado por el JR3
            struct TorqueVector {
                double mx, my, mz;
            } _initT; // vector de momento proporcionado por el JR3
        } _jr3;

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

        double _rzmp, _rWorkSpace, _normVector, _modFS, _modFF, angle, _thetaX, _thetaY;
        std::vector<double> quat, quatC, preFF, FF, preFM, FM; // quaternios


        std::vector<double> currentQ;
        std::vector<double> beforeQ;
        std::vector<double> currentX;

        /** Set INITIAL POS-VEL-ACC **/                         bool preprogrammedInitTrajectory();
        /** ARM CONTROL WITH A VELOCITY STRATEGY **/            void strategyVelocity(Bottle& FTsensor);
        /** ARM CONTROL WITH A POSITION STRATEGY **/            void strategyPositionDirect(Bottle& FTsensor);
        /** Callback on incoming Bottle. **/            virtual void onRead(Bottle& FTsensor);
        /** Reading from the FT_JR3_sensor. **/                 void ReadFTSensor(Bottle& FTsensor);
        /** Rotation Transformation matrix of JR3. **/          void AxesTransform1();
        /** Transformation matrix between JR3 and tray. **/     void AxesTransform2();
        /** Calculating ZMP of the bottle. **/                  void ZMPcomp();
        /** Control based on the 3D-LIMP. **/                   void LIPM3d();
        /** Saving the ZMP measurements. **/                    void saveToFile();
        /** Configurating the pose and F/t references. **/      void poseRefCalculate(Bottle& FTsensor);
        /** Offset JR3 measurements. **/                        void offSetJR3(Bottle& FTsensor);

        //-- Robot device
        yarp::dev::IEncoders *iEncoders;
        yarp::dev::IPositionControl *iPositionControl;
        yarp::dev::IPositionDirect *iPositionDirect;
        yarp::dev::IVelocityControl *iVelocityControl;

        //-- Solver device
        teo::ICartesianSolver *iCartesianSolver;

};

}  // namespace teo

#endif // __IN_SR_PORT_HPP__
