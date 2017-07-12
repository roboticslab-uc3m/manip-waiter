// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
el test 2 está asociado para el journal SENSORS en el que el topic del paper es el modelo de
control de la botella basado en rampas variables usando sensores de fuerza/par

el test 2 consiste en observar y comprobar como ante variaciones de la orientacion del
sensor jr3, volvemos a obtener un nuevo valor del vector de fuerza F (fx, fy, fz),
que realmente coincide con las fuerzasd aplicadas posteriormente al ZMP. Este test
se debe dividir a mano en otros tres, en el que se medirá el ZMP, tanto en el plano
frontal (0,1,0), como en el plano sagital (1,0,0), como en un plano combiancion
de los dos anteriores (1,1,0).
*/

#include "Test2.hpp"

namespace roboticslab
{

/************************************************************************/
bool Test2::configure(ResourceFinder &rf) {

    std::string remote = rf.check("remote",yarp::os::Value(DEFAULT_REMOTE),"remote robot to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("Test2 options:\n");
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
    std::string localStr("/test2/");
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
        return false;    }
    if( ! solverDevice.view(iCartesianSolver) )    {
        CD_ERROR("Could not view iCartesianSolver in: %s.\n",solverStr.c_str());
        return false;    }
    //inCvPort.setICartesianSolver(iCartesianSolver);
    inSrPort.setICartesianSolver(iCartesianSolver);
    
    //-----------------OPEN LOCAL PORTS------------//
    //inCvPort.useCallback();
    inSrPort.useCallback();
    //inCvPort.open("/test2/jr3/ch3:i");
    inSrPort.open("/test2/jr3/ch3:i");

    return true;
}

/************************************************************************/
double Test2::getPeriod() {
    return 2.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool Test2::updateModule() {
    //printf("StateMachine in state [%d]. Test2 alive...\n", stateMachine.getMachineState());
    //printf("StateMachine in alive.\n");
    return true;
}

/************************************************************************/
bool Test2::interruptModule() {
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
