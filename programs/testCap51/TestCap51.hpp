// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FM_TESTCAP51_HPP__ 
#define __FM_TESTCAP51_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>

#include "InSrPort.hpp"

#define DEFAULT_ROBOT "/teo"
// Por defecto, usaremos el robot -> con simulador: "/teoSim"

using namespace yarp::os;

namespace roboticslab
{

/**
 * @ingroup testCap51
 *
 * @brief testCap51.
 *
 */
class TestCap51 : public RFModule {
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

#endif // __FM_TESTCAP51_HPP__
