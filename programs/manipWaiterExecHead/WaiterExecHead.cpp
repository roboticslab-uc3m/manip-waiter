// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WaiterExecHead.hpp"
#include "InCvPort.hpp"

namespace teo
{

/************************************************************************/

bool WaiterExecHead::configure(ResourceFinder &rf) {

    //ConstString fileName(DEFAULT_FILE_NAME);
    
    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("WaiterExecHead options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        //printf("\t--file (default: \"%s\")\n",fileName.c_str());
    }
    //if (rf.check("file")) fileName = rf.find("file").asString();
    //printf("WaiterExecHead using file: %s\n",fileName.c_str());

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    //
    Property headOptions;
    headOptions.put("device","remote_controlboard");
    headOptions.put("local","/waiterExecHead/head");
    headOptions.put("remote","/teo/head");
    headDevice.open(headOptions);
    if( ! headDevice.isValid() ) {
        printf("head remote_controlboard instantiation not worked.\n");
        return false;
    }

//    yarp::dev::IVelocityControl *iVelocityControl;
    if( ! headDevice.view(iVelocityControl) ) {
        printf("view(iVelocityControl) not worked.\n");
        return false;
    }
    inCvPort.setIVelocityControl(iVelocityControl);
    iVelocityControl->setVelocityMode();
    inCvPort.setIVelocityControl(iVelocityControl);
    //-----------------OPEN LOCAL PORTS------------//
    inSrPort.setInCvPortPtr(&inCvPort);
    inCvPort.useCallback();
    inSrPort.useCallback();
    inSrPort.open("/waiterExecHead/DialogueManager/command:i");
    inCvPort.open("/waiterExecHead/cvBottle/state:i");

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
