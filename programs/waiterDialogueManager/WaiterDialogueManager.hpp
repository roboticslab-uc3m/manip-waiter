// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FM_DIALOGUE_MANAGER_HPP__
#define __FM_DIALOGUE_MANAGER_HPP__

#include <yarp/os/all.h>
#include <stdlib.h>

#include "StateMachine.hpp"

#define DEFAULT_LANGUAGE "english"

//instrucciones para el followme
#define VOCAB_FOLLOW_ME VOCAB4('f','o','l','l')
#define VOCAB_STOP_FOLLOWING VOCAB4('s','f','o','l')

//insatrucciones para el waiterbot
#define VOCAB_HELLO_TEO VOCAB4('e','l','o','t')
#define VOCAB_GO_TEO VOCAB4('g','t','e','o')
#define VOCAB_WATER_PLEASE VOCAB4('w','p','l','e')
#define VOCAB_STOP_TEO VOCAB4('s','t','e','o')

namespace teo
{

/**
 * @ingroup WaiterDialogueManager
 *
 * @brief Waiter Dialogue Manager.
 */
class WaiterDialogueManager : public yarp::os::RFModule {
  private:
    StateMachine stateMachine;
    yarp::os::BufferedPort<yarp::os::Bottle> inSrPort;
    yarp::os::Port outTtsPort;
    yarp::os::Port outCmdPortManip;
    yarp::os::Port outCmdPortHead;

    bool interruptModule();
    double getPeriod();
    bool updateModule();

  public:
    bool configure(yarp::os::ResourceFinder &rf);
};

}  // namespace teo

#endif  // __FM_DIALOGUE_MANAGER_HPP__
