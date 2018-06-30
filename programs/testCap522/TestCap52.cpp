// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
el test 52 está asociado para el journal SENSORS en el que el topic del paper es el modelo de
control de la botella basado en rampas variables usando sensores de fuerza/par

el test 52 consiste en observar y comprobar como ante variaciones de la orientacion del
sensor jr3, volvemos a obtener un nuevo valor del vector de fuerza F (fx, fy, fz),
que realmente coincide con las fuerzas aplicadas posteriormente al ZMP. Este test
se debe dividir a mano en otros tres, en el que se medirá el ZMP, tanto en el plano
frontal (0,1,0), como en el plano sagital (1,0,0), como en un plano combiancion
de los dos anteriores (1,1,0).
*/

#include "TestCap52.hpp"

namespace roboticslab
{

/************************************************************************/
bool TestCap52::configure(ResourceFinder &rf) {

    std::string remote = rf.check("remote",yarp::os::Value(DEFAULT_REMOTE),"remote robot to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("Test3 options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--remote ('teo' or 'teoSim')\n");
    }
    printf("Test1 using remote: %s [%s]\n",remote.c_str(),DEFAULT_REMOTE);

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    //-- Robot device
    Property leftArmOptions;
    leftArmOptions.put("device","remote_controlboard");
    std::string localStr("/test3/");
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
        return false;    }
    if( ! leftArmDevice.view(iEncoders) )    {
        printf("view(iEncoders) not worked.\n");
        return false;    }
    if( ! leftArmDevice.view(iPositionControl) )    {
        printf("view(iPositionControl) not worked.\n");
        return false;    }
    if( ! leftArmDevice.view(iPositionDirect) )    {
        printf("view(iPositionDirect) not worked.\n");
        return false;    }
    if( ! leftArmDevice.view(iVelocityControl) )    {
        printf("view(iVelocityControl) not worked.\n");
        return false;    }

    myRateThread.setIEncodersControl(iEncoders);
    myRateThread.setIPositionControl(iPositionControl);
    myRateThread.setIPositionDirect(iPositionDirect);
    myRateThread.setIVelocityControl(iVelocityControl);

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
    myRateThread.setICartesianSolver(iCartesianSolver);
    
    //-----------------OPEN LOCAL PORTS------------//
    myRateThread.jr3.open("/test3/jr3/ch3:i");
    myRateThread.inertial.open("/test3/inertial:i");

    //--- start rateThread
    myRateThread.start();

    return true;
}

/************************************************************************/
double TestCap52::getPeriod() {
    return 2.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool TestCap52::updateModule() {
    //printf("StateMachine in state [%d]. Test3 alive...\n", stateMachine.getMachineState());
    //printf("StateMachine in alive.\n");
    return true;
}

/************************************************************************/
bool TestCap52::interruptModule() {
    printf("Test1 closing...\n");
    myRateThread.jr3.interrupt();
    myRateThread.inertial.interrupt();
    myRateThread.jr3.close();
    myRateThread.inertial.close();

    solverDevice.close();
    leftArmDevice.close();
    return true;
}

/************************************************************************/
} // namespace roboticslab
