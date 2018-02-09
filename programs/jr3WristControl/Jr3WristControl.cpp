// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3WristControl.hpp"

namespace roboticslab
{

/************************************************************************/
bool Jr3WristControl::configure(ResourceFinder &rf) {

    std::string robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"name of /robot to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("Jr3WristControl options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--robot ('teo' or 'teoSim')\n");
        printf("\t--robot: %s [%s]\n",robot.c_str(),DEFAULT_ROBOT);
    }
    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }
    std::string waiterStr("/waiter");

    //-----------------LEFT ARM------------//
    yarp::os::Property leftArmOptions;
    leftArmOptions.put("device","remote_controlboard");
    leftArmOptions.put("remote",robot+"/leftArm");
    leftArmOptions.put("local",waiterStr+robot+"/leftArm");
    leftArmDevice.open(leftArmOptions);
    if(!leftArmDevice.isValid()) {
        printf("robot leftArm device not available.\n");
        leftArmDevice.close();
        yarp::os::Network::fini();
        return false;
    }

    if (!leftArmDevice.view(leftArmIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring leftArmPos interface\n");
        return false;
    } else printf("[success] Acquired leftArmPos interface\n");

    if (!leftArmDevice.view(leftArmIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring leftArmIControlMode2 interface\n");
        return false;
    } else printf("[success] Acquired leftArmIControlMode2 interface\n");

    if (!leftArmDevice.view(iEncoders) ) {
        printf("[warning] Problems acquiring iEncoders interface\n");
        return false;
    } else printf("[success] Acquired iEncoders interface\n");

     //-- Set control modes
    int leftArmAxes;
    leftArmIPositionControl2->getAxes(&leftArmAxes);
    std::vector<int> leftArmControlModes(leftArmAxes,VOCAB_CM_POSITION);
    if(! leftArmIControlMode2->setControlModes( leftArmControlModes.data() )){
        printf("[warning] Problems setting position control mode of: left-arm\n");
        return false;
    }
    /*inCvPort.setIEncodersControl(iEncoders);
    inCvPort.setIPositionControl(iPositionControl);
    inCvPort.setIPositionDirect(iPositionDirect);
    inCvPort.setIVelocityControl(iVelocityControl);*/
    
    inSrPort.setIEncodersControl(iEncoders);
    inSrPort.setIPositionControl(leftArmIPositionControl2);
    //inSrPort.setIPositionDirect(iPositionDirect);
    //inSrPort.setIVelocityControl(iVelocityControl);

    //-- Solver device
    yarp::os::Property solverOptions;
    solverOptions.fromString( rf.toString() );
    std::string solverStr = "KdlSolver";
    solverOptions.put("device",solverStr);
    solverDevice.open(solverOptions);

    if( ! solverDevice.isValid() )    {
        CD_ERROR("solver device not valid: %s.\n",solverStr.c_str());
        return false;    }
    if( ! solverDevice.view(iCartesianSolver) )    {
        CD_ERROR("Could not view iCartesianSolver in: %s.\n",solverStr.c_str());
        return false;    }
    //inCvPort.setICartesianSolver(iCartesianSolver);
    //inSrPort.setICartesianSolver(iCartesianSolver);
    
    //-----------------OPEN LOCAL PORTS------------//
    //inCvPort.useCallback();
    inSrPort.useCallback();
    //inCvPort.open("/jr3WristControl/jr3/ch3:i");
    inSrPort.open("/jr3WristControl/jr3/ch2:i"); //ch2:i rightArm - ch3:i leftArm

    return true;
}

/************************************************************************/
double Jr3WristControl::getPeriod() {
    return 2.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool Jr3WristControl::updateModule() {
    //printf("StateMachine in state [%d]. Jr3WristControl alive...\n", stateMachine.getMachineState());
    //printf("StateMachine in alive.\n");
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
} // namespace roboticslab
