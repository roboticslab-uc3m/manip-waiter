// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __IN_CV_PORT_HPP__
#define __IN_CV_PORT_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>

#include <fstream>
#include <stdio.h>

#include <math.h>  //-- fabs

#include "ICartesianSolver.h"

#include "ColorDebug.hpp"
#include <yarp/dev/IPositionDirect.h>

//instrucciones para el followme
#define VOCAB_FOLLOW_ME VOCAB4('f','o','l','l')
#define VOCAB_STOP_FOLLOWING VOCAB4('s','f','o','l')

//insatrucciones para el waiterbot
#define VOCAB_HELLO_TEO VOCAB4('e','l','o','t')
#define VOCAB_GO_TEO VOCAB4('g','t','e','o')
#define VOCAB_WATER_PLEASE VOCAB4('w','p','l','e')
#define VOCAB_STOP_TEO VOCAB4('s','t','e','o')

#define DEFAULT_QDOT_LIMIT 10

#define DEFAULT_STRATEGY "positionDirect"
//#define DEFAULT_STRATEGY "velocity"


using namespace yarp::os;

namespace teo
{

/**
 * @ingroup Jr3WristControl
 *
 * @brief Input port of computer vision data.
 *
 */
class InCvPort : public BufferedPort<Bottle> {
    public:

        InCvPort()        {
            follow = 0;
            a = 0;
            pepinito = 1;
        }

        void setIEncodersControl(yarp::dev::IEncoders *iEncoders) {
            this->iEncoders = iEncoders;
        }
        void setIPositionControl(yarp::dev::IPositionControl *iPositionControl) {
            this->iPositionControl = iPositionControl;
        }
        void setIPositionDirect(yarp::dev::IPositionDirect *iPositionDirect) {
            this->iPositionDirect = iPositionDirect;
        }
        void setIVelocityControl(yarp::dev::IVelocityControl *iVelocityControl) {
            this->iVelocityControl = iVelocityControl;
        }
        void setICartesianSolver(teo::ICartesianSolver *iCartesianSolver) {
            this->iCartesianSolver = iCartesianSolver;
        }

        void setFollow(int value);

        yarp::os::Port port2;
        yarp::os::Port port3;

    private:

        int follow;
        int a;
        int numRobotJoints;
        int pepinito;
        double initpos;

        struct SensorData {
            double fx, fy, fz;
            double mx, my, mz;
        } _sensor3;

        struct TrayData {
            double fx, fy, fz;
            double mx, my, mz;
            double xzmp, yzmp;
        } _tray;

        double _rzmp;
        float _d;  //distance in mm between the plate center and the sensor center in the X axis

        std::vector<double> currentQ;
        std::vector<double> beforeQ;

        /** Set left ARM INITIAL POSITION **/
        bool preprogrammedInitTrajectory();

        /** left ARM CONTROL WITH A VELOCITY STRATEGY **/
        void strategyVelocity(Bottle& FTsensor);

        /** left ARM CONTROL WITH A POSITION STRATEGY **/
        void strategyPositionDirect(Bottle& FTsensor);

        /** Callback on incoming Bottle. **/
        virtual void onRead(Bottle& FTsensor);

        /** Reading from the FT_JR3_sensor. **/
        void ReadFTSensor(Bottle& FTsensor);

        /** Transformation matrix between TEO_body_axes (world) and Jr3_axes with horizontal tray (waiter). **/
        void AxesTransform();

        /** Calculating ZMP of the bottle. **/
        void ZMPcomp();

        /** Control based on the 3D-LIMP. **/
        void LIPM3d();

        //-- Robot device
        yarp::dev::IEncoders *iEncoders;
        yarp::dev::IPositionControl *iPositionControl;
        yarp::dev::IPositionDirect *iPositionDirect;
        yarp::dev::IVelocityControl *iVelocityControl;

        //-- Solver device
        teo::ICartesianSolver *iCartesianSolver;

};

}  // namespace teo

#endif // __IN_CV_PORT_HPP__
