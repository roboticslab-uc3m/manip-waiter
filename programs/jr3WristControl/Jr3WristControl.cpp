// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3WristControl.hpp"

namespace teo
{

/************************************************************************/
bool Jr3WristControl::configure(ResourceFinder &rf) {

    std::string remote = rf.check("remote",yarp::os::Value(DEFAULT_REMOTE),"remote robot to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("Jr3WristControl options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--remote ('teo' or 'teoSim')\n");
    }
    printf("Jr3WristControl using remote: %s [%s]\n",remote.c_str(),DEFAULT_REMOTE);

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    //-- Robot device
    Property leftArmOptions;
    leftArmOptions.put("device","remote_controlboard");
    std::string localStr("/jr3WristControl/");
    localStr += remote;
    localStr += "/leftArm";
    leftArmOptions.put("local",localStr);
    std::string remoteStr("/");
    remoteStr += remote;
    remoteStr += "/leftArm";
    leftArmOptions.put("remote",remoteStr);
    leftArmDevice.open(leftArmOptions);

    if( ! leftArmDevice.isValid() )    {
        printf("leftArm remote_controlboard instantiation not worked.\n");
        return false;
    }
    if( ! leftArmDevice.view(iEncoders) )    {
        printf("view(iEncoders) not worked.\n");
        return false;
    }
    if( ! leftArmDevice.view(iPositionControl) )    {
        printf("view(iPositionControl) not worked.\n");
        return false;
    }
    if( ! leftArmDevice.view(iPositionDirect) )    {
        printf("view(iPositionDirect) not worked.\n");
        return false;
    }
    if( ! leftArmDevice.view(iVelocityControl) )    {
        printf("view(iVelocityControl) not worked.\n");
        return false;
    }

    /*inCvPort.setIEncodersControl(iEncoders);
    inCvPort.setIPositionControl(iPositionControl);
    inCvPort.setIPositionDirect(iPositionDirect);
    inCvPort.setIVelocityControl(iVelocityControl);*/
    
    inSrPort.setIEncodersControl(iEncoders);
    inSrPort.setIPositionControl(iPositionControl);
    inSrPort.setIPositionDirect(iPositionDirect);
    inSrPort.setIVelocityControl(iVelocityControl);

    //-- Solver device
    yarp::os::Property solverOptions;
    solverOptions.fromString( rf.toString() );
    std::string solverStr = "KdlSolver";
    solverOptions.put("device",solverStr);
    solverDevice.open(solverOptions);

    if( ! solverDevice.isValid() )    {
        CD_ERROR("solver device not valid: %s.\n",solverStr.c_str());
        return false;
    }
    if( ! solverDevice.view(iCartesianSolver) )    {
        CD_ERROR("Could not view iCartesianSolver in: %s.\n",solverStr.c_str());
        return false;
    }
    //inCvPort.setICartesianSolver(iCartesianSolver);
    inSrPort.setICartesianSolver(iCartesianSolver);
    
    //-----------------OPEN LOCAL PORTS------------//
    //inCvPort.useCallback();
    inSrPort.useCallback();
    //inCvPort.open("/jr3WristControl/jr3/ch3:i");
    inSrPort.open("/jr3WristControl/jr3/ch3:i");

    return true;
}

/************************************************************************/
double Jr3WristControl::getPeriod() {
    return 1.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool Jr3WristControl::updateModule() {
    //printf("StateMachine in state [%d]. Jr3WristControl alive...\n", stateMachine.getMachineState());
    return true;
}

/************************************************************************/
bool Jr3WristControl::interruptModule() {
    printf("Jr3WristControl closing...\n");
    //inCvPort.disableCallback();
    inSrPort.disableCallback();
    //inCvPort.interrupt();
    inSrPort.interrupt();
    //inCvPort.close();
    inSrPort.close();

    solverDevice.close();
    leftArmDevice.close();
    return true;
}

/************************************************************************/
} // namespace teo
