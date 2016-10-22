// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __STATE_MACHINE__
#define __STATE_MACHINE__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>

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
 * @ingroup waiterDialogueManager
 *
 * @brief implements a specific state machine for waiterDialogueManager.
 */
class StateMachine : public yarp::os::Thread {
protected:

    yarp::os::BufferedPort<yarp::os::Bottle> *inSrPort;
    yarp::os::Port *outCmdPortHead;
    yarp::os::Port *outCmdPortManip;
    yarp::os::Port *outTtsPort;

    int _machineState;

    yarp::os::ConstString _inStrState1;

    void ttsSay(const yarp::os::ConstString& sayConstString);
    yarp::os::ConstString asrListen();

    // output variables
    std::string notUnderstand;
    std::string repeat;
    std::string hello;
    std::string drink;
    std::string take;
    std::string finish;
    // input variables
    std::string hiTeo;
    std::string goOnTeo;
    std::string waterPlease;
    std::string stopNow;

public:

    /**
     * Initialization method. The thread executes this function
     * when it starts and before "run". This is a good place to 
     * perform initialization tasks that need to be done by the 
     * thread itself (device drivers initialization, memory 
     * allocation etc). If the function returns false the thread 
     * quits and never calls "run". The return value of threadInit()
     * is notified to the class and passed as a parameter 
     * to afterStart(). Note that afterStart() is called by the 
     * same thread that is executing the "start" method.
     */
    bool threadInit();

    /**
     * Loop function. This is the thread itself.
     */
    void run();

    /**
     * Get its state.
     */
    int getMachineState();

    /** Register an input callback port for asr. */
    void setInSrPort(yarp::os::BufferedPort<yarp::os::Bottle>* inSrPort);

    /** Register an output Port for commands for the head. */
    void setOutCmdPortHead(yarp::os::Port* outCmdPort);

    /** Register an output Port for commands for the left arm. */
    void setOutCmdPortManip(yarp::os::Port* outCmdPort);

    /** Register an output Port for tts. */
    void setOutTtsPort(yarp::os::Port* outTtsPort);

    bool setLanguage(std::string language);

};

}  // namespace teo

#endif

