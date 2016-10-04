// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __WAITER_EXEC_HEAD_HPP__
#define __WAITER_EXEC_HEAD_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>

#include "InCvPort.hpp"
#include "InSrPort.hpp"

//instrucciones para el followme
#define VOCAB_FOLLOW_ME VOCAB4('f','o','l','l')
#define VOCAB_STOP_FOLLOWING VOCAB4('s','f','o','l')

//insatrucciones para el waiterbot
#define VOCAB_HELLO_TEO VOCAB4('e','l','o','t')
#define VOCAB_GO_TEO VOCAB4('g','t','e','o')
#define VOCAB_WATER_PLEASE VOCAB4('w','p','l','e')
#define VOCAB_STOP_TEO VOCAB4('s','t','e','o')

using namespace yarp::os;
using namespace yarp::dev;

namespace teo
{

/**
 * @ingroup WaiterExecHead
 *
 * @brief Waiter Exec Head.
 *
 */
class WaiterExecHead : public RFModule {
    public:
        bool configure(ResourceFinder &rf);

    protected:
        InSrPort inSrPort;
        InCvPort inCvPort;
        yarp::dev::PolyDriver headDevice;
        yarp::dev::IPositionControl *iPositionControl;
        yarp::dev::IVelocityControl *iVelocityControl;

        bool interruptModule();
        double getPeriod();
        bool updateModule();

};

}  // namespace teo

#endif  // __WAITER_EXEC_HEAD_HPP__
