// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WaiterExecHead.hpp"
#include "InCvPort.hpp"

namespace teo
{

/************************************************************************/

bool WaiterExecHead::configure(ResourceFinder &rf) {

    std::string remote = rf.check("remote",yarp::os::Value(DEFAULT_REMOTE),"remote robot to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("WaiterExecHead options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--remote ('teo' or 'teoSim')\n");
    }
    printf("WaiterExecManip using remote: %s [%s]\n",remote.c_str(),DEFAULT_REMOTE);

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    //
    Property headOptions;
    headOptions.put("device","remote_controlboard");
    std::string localStr("/manipWaiterExecHead/");
    localStr += remote;
    localStr += "/head";
    headOptions.put("local",localStr);
    std::string remoteStr("/");
    remoteStr += remote;
    remoteStr += "/head";
    headOptions.put("remote",remoteStr);
    headDevice.open(headOptions);
    if( ! headDevice.isValid() ) {
        printf("head remote_controlboard instantiation not worked.\n");
        return false;
    }

    if( ! headDevice.view(iPositionControl) ) {
        printf("view(iPositionControl) not worked.\n");
        return false;
    }
    inCvPort.setIPositionControl(iPositionControl);
    iPositionControl->setPositionMode();
/*
    if( ! headDevice.view(iVelocityControl) ) {
        printf("view(iVelocityControl) not worked.\n");
        return false;
    }
    inCvPort.setIVelocityControl(iVelocityControl);

    iVelocityControl->setVelocityMode();
*/


    //-----------------OPEN LOCAL PORTS------------//
    inSrPort.setInCvPortPtr(&inCvPort);
    inCvPort.useCallback();
    inSrPort.useCallback();
    inSrPort.open("/manipWaiterExecHead/DialogueManager/command:i");
    inCvPort.open("/manipWaiterExecHead/cvBottle/state:i");

    return true;
}

/************************************************************************/
double WaiterExecHead::getPeriod() {
    return 2.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool WaiterExecHead::updateModule() {
    //printf("StateMachine in state [%d]. WaiterExecHead alive...\n", stateMachine.getMachineState());
    return true;
}

/************************************************************************/

bool WaiterExecHead::interruptModule() {
    printf("WaiterExecHead closing...\n");
    inCvPort.disableCallback();
    inSrPort.disableCallback();
    inCvPort.interrupt();
    inSrPort.interrupt();
    inCvPort.close();
    inSrPort.close();
    return true;
}

/************************************************************************/

}  // namespace teo
