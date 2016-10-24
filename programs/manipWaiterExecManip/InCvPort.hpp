// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __IN_CV_PORT_HPP__
#define __IN_CV_PORT_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>

#include <fstream>
#include <stdio.h>
#include <math.h>

//instrucciones para el followme
#define VOCAB_FOLLOW_ME VOCAB4('f','o','l','l')
#define VOCAB_STOP_FOLLOWING VOCAB4('s','f','o','l')

//insatrucciones para el waiterbot
#define VOCAB_HELLO_TEO VOCAB4('e','l','o','t')
#define VOCAB_GO_TEO VOCAB4('g','t','e','o')
#define VOCAB_WATER_PLEASE VOCAB4('w','p','l','e')
#define VOCAB_STOP_TEO VOCAB4('s','t','e','o')

using namespace yarp::os;

namespace teo
{

/**
 * @ingroup followMeExecutionCore
 *
 * @brief Input port of computer vision data.
 *
 */
class InCvPort : public BufferedPort<Bottle> {
    public:

        InCvPort()
        {
            follow = 0;
            a = 0;
            coordY = 0.347;
        }

        void setIPositionControl(yarp::dev::IPositionControl *iPositionControl) {
            this->iPositionControl = iPositionControl;
        }

        void setFollow(int value);
        void setOutPort(yarp::os::Port *_pOutPort);
        yarp::os::Port *pOutPort;

protected:
        int follow;
        int a;
        int c;
        int i;
        double coordY, angle, dist;
        /** Callback on incoming Bottle. **/
        virtual void onRead(Bottle& b);

        yarp::dev::IPositionControl *iPositionControl;

        yarp::dev::PolyDriver robotDevice;
        yarp::dev::IEncoders *iEncoders;
//        yarp::dev::IPositionControl *iPositionControl;
        yarp::dev::IVelocityControl *iVelocityControl;
        yarp::dev::IControlLimits *iControlLimits;
        yarp::dev::ITorqueControl *iTorqueControl;

        int numRobotJoints, numSolverLinks;

        /** State encoded as a VOCAB which can be stored as an int */
        int currentState;

        int getCurrentState();
        void setCurrentState(int value);
        yarp::os::Semaphore currentStateReady;

        /** MOVL keep track of movement start time to know at what time of trajectory movement we are */
        double movementStartTime;

        /** MOVV desired Cartesian velocity */
        std::vector<double> xdotd;

        /** FORC desired Cartesian force */
        std::vector<double> td;


};

}  // namespace teo

#endif  // __IN_CV_PORT_HPP__
