// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WaiterExecManip.hpp"

namespace teo
{

/************************************************************************/

bool WaiterExecManip::configure(ResourceFinder &rf) {

    //ConstString fileName(DEFAULT_FILE_NAME);
    
    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("WaiterExecManip options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        //printf("\t--file (default: \"%s\")\n",fileName.c_str());
    }
    //if (rf.check("file")) fileName = rf.find("file").asString();
    //printf("WaiterExecManip using file: %s\n",fileName.c_str());

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    //
    Property leftArmOptions;
    leftArmOptions.put("device","remote_controlboard");
    leftArmOptions.put("local","/waiterExecManip/teo/leftArm");
    leftArmOptions.put("remote","/teo/leftArm");
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

    //-----------------OPEN LOCAL PORTS------------//
    inSrPort.setInCvPortPtr(&inCvPort);
    inCvPort.useCallback();
    inSrPort.useCallback();
    inSrPort.open("/waiterExecManip/DialogueManager/command:i");
    inCvPort.open("/waiterExecManip/cvBottle/state:i");

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
    return true;
}

/************************************************************************/

}  // namespace teo
