// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
el test 1 está asociado para el journal SENSORS en el que el topic del paper es el modelo de
control de la botella basado en rampas variables usando sensores de fuerza/par

el test 1 consiste en observar y comprobar si el ZMP se calcula correctamente en funcion del
angulo de inclinacion de la bandeja. Este test se debe dividir a mano en otros tres, en el que
se medirá el ZMP, tanto en el plano frontal (0,1,0), como en el plano sagital (1,0,0), como en un plano que sea combinacion
de los dos anteriores (1,1,0).
*/

#include "Test1.hpp"

namespace roboticslab
{

/************************************************************************/
bool Test1::configure(ResourceFinder &rf) {

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
    if (!leftArmDevice.view(leftArmIEncoders) ) {
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

    //-- Conection between Jr3WristControl & inSrPort
    inSrPort.setIEncodersControl(leftArmIEncoders);
    inSrPort.setIPositionControl2(leftArmIPositionControl2);
    inSrPort.setIVelocityControl2(leftArmIVelocityControl2);

    //-- Solver device
    yarp::os::Property solverOptions;
    solverOptions.fromString( rf.toString() );
    std::string solverStr = "KdlSolver";
    solverOptions.put("device",solverStr);
    solverOptions.put("angleRepr","axisAngle");
    solverDevice.open(solverOptions);

    if( ! solverDevice.isValid() )    {
        CD_ERROR("solver device not valid: %s.\n",solverStr.c_str());
        return false;    }
    if( ! solverDevice.view(iCartesianSolver) )    {
        CD_ERROR("Could not view iCartesianSolver in: %s.\n",solverStr.c_str());
        return false;    }
    inSrPort.setICartesianSolver(iCartesianSolver);
    
    //-----------------OPEN LOCAL PORTS------------//
    inSrPort.useCallback();
    inSrPort.open("/test1/jr3/ch3:i");

    return true;
}

/************************************************************************/
double Test1::getPeriod() {
    return 2.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool Test1::updateModule() {
    //printf("StateMachine in state [%d]. Test1 alive...\n", stateMachine.getMachineState());
    //printf("StateMachine in alive.\n");
    return true;
}

/************************************************************************/
bool Test1::interruptModule() {
    printf("Test1 closing...\n");
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
