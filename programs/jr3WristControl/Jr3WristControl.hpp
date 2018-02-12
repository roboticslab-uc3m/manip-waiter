// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FM_JR3_WRIST_CONTROL_HPP__
#define __FM_JR3_WRIST_CONTROL_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>

//#include "InCvPort.hpp"
#include "InSrPort.hpp"

#define DEFAULT_ROBOT "/teo"

//instrucciones para el followme
//#define VOCAB_FOLLOW_ME VOCAB4('f','o','l','l')
//#define VOCAB_STOP_FOLLOWING VOCAB4('s','f','o','l')

//insatrucciones para el waiterbot
//#define VOCAB_HELLO_TEO VOCAB4('e','l','o','t')
//#define VOCAB_GO_TEO VOCAB4('g','t','e','o')
//#define VOCAB_WATER_PLEASE VOCAB4('w','p','l','e')
//#define VOCAB_STOP_TEO VOCAB4('s','t','e','o')


using namespace yarp::os;

namespace roboticslab
{

/**
 * @ingroup Jr3WristControl
 *
 * @brief Jr3 Wrist Control.
 *
 */
class Jr3WristControl : public RFModule {
    public:
        bool configure(ResourceFinder &rf);

    protected:

        InSrPort inSrPort;

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
        yarp::dev::PolyDriver solverDevice;
        roboticslab::ICartesianSolver *iCartesianSolver;

        bool interruptModule();
        double getPeriod();
        bool updateModule();

};

}  // namespace roboticslab

#endif // __FM_JR3_WRIST_CONTROL_HPP__
