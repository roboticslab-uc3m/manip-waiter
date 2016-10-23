// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WaiterExecManip.hpp"

namespace teo
{

/************************************************************************/

bool WaiterExecManip::configure(ResourceFinder &rf) {

    std::string remote = rf.check("remote",yarp::os::Value(DEFAULT_REMOTE),"remote robot to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("WaiterExecManip options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--remote ('teo' or 'teoSim')\n");
    }
    printf("WaiterExecManip using remote: %s [%s]\n",remote.c_str(),DEFAULT_REMOTE);

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    //
    Property leftArmOptions;
    leftArmOptions.put("device","remote_controlboard");
    std::string localStr("/manipWaiterExecManip/");
    localStr += remote;
    localStr += "/leftArm";
    leftArmOptions.put("local",localStr);
    std::string remoteStr("/");
    remoteStr += remote;
    remoteStr += "/leftArm";
    leftArmOptions.put("remote",remoteStr);
    leftArmDevice.open(leftArmOptions);
    if( ! leftArmDevice.isValid() ) {
        printf("leftArm remote_controlboard instantiation not worked.\n");
        return false;
    }
    if( ! leftArmDevice.view(iPositionControl) ) {
        printf("view(iPositionControl) not worked.\n");
        return false;
    }
    inCvPort.setIPositionControl(iPositionControl);

    inCvPort.setOutPort(&outPort);

    //-----------------OPEN LOCAL PORTS------------//
    inSrPort.setInCvPortPtr(&inCvPort);
    inCvPort.useCallback();
    inSrPort.useCallback();
    inSrPort.open("/manipWaiterExecManip/DialogueManager/command:i");
    inCvPort.open("/manipWaiterExecManip/cvBottle/state:i");
    outPort.open("/manipWaiterExecManip/leftArm/CartesianControl/rpc:c");

    return true;
}

/************************************************************************/
double WaiterExecManip::getPeriod() {
    return 2.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool WaiterExecManip::updateModule() {
    //printf("StateMachine in state [%d]. FollowMeExecutionCore alive...\n", stateMachine.getMachineState());
    return true;
}

/************************************************************************/

bool WaiterExecManip::interruptModule() {
    printf("WaiterExecManip closing...\n");
    inCvPort.disableCallback();
    inSrPort.disableCallback();
    inCvPort.interrupt();
    inSrPort.interrupt();
    inCvPort.close();
    inSrPort.close();
    outPort.close();
    return true;
}

/************************************************************************/

}  // namespace teo
