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

#define DEFAULT_QDOT_LIMIT 10

#define DEFAULT_STRATEGY "positionDirect"
//#define DEFAULT_STRATEGY "velocity"


using namespace yarp::os;

namespace roboticslab
{

/**
 * @ingroup Jr3WristControl
 *
 * @brief Input port of Force/Torque data.
 *
 */
class InCvPort : public BufferedPort<Bottle> {
    public:

        InCvPort()        {
            follow = 0;
            a = 0;
            pepinito = 1;
            _d = 0.025;
            iteration=1;
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
        void setICartesianSolver(roboticslab::ICartesianSolver *iCartesianSolver) {
            this->iCartesianSolver = iCartesianSolver;
        }

        void setFollow(int value);

        //yarp::os::Port port2; posibilidad de usar la muñeca izquierda
        yarp::os::Port port3;

    private:

        int follow;
        int a;
        int numRobotJoints;
        int pepinito;
        double initpos;
        int iteration;

        struct SensorData {
            double fx, fy, fz;
            double mx, my, mz;
        } _sensor3;

        struct TrayData {
            double fx, fy, fz;
            double mx, my, mz;
            double xzmp, yzmp;
        } _tray;

        double _rzmp, _rWorkSpace;
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

        /** Saving the ZMP measurements. **/
        void saveToFile();

        //-- Robot device
        yarp::dev::IEncoders *iEncoders;
        yarp::dev::IPositionControl *iPositionControl;
        yarp::dev::IPositionDirect *iPositionDirect;
        yarp::dev::IVelocityControl *iVelocityControl;

        //-- Solver device
        roboticslab::ICartesianSolver *iCartesianSolver;

};

}  // namespace roboticslab

#endif // __IN_CV_PORT_HPP__
