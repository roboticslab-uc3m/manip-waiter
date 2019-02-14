// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**

el test 53-a es una extension minima del 53. los unicos cambios pertinentes son los
relacionados con el modo de control del robot.

Para este test se activa el modo "VOCAB_CM_VELOCITY" para actuar sobre los tobillos
con comandos de velocidad.

puede ser el inicio para ampliar el push recovery a la estrategia HIP,

**/

#include "TestCap53a.hpp"

namespace roboticslab
{

/************************************************************************/
bool TestCap53a::configure(ResourceFinder &rf) {

    std::string robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"name of /robot to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("testCap53a options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--robot ('teo' or 'teoSim')\n");
        printf("\t--robot: %s [%s]\n",robot.c_str(),DEFAULT_ROBOT);
    }
    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }
    std::string waiterStr("/bodyBal");
    /** **************************************************************************************
     * ******************************************************************************** **/


    // ------ TRUNK DEV -------
    yarp::os::Property trunkOptions;
    trunkOptions.put("device","remote_controlboard");
    trunkOptions.put("remote","/teo/trunk");
    trunkOptions.put("local",waiterStr+robot+"/trunk");
    trunkDevice.open(trunkOptions);
    if(!trunkDevice.isValid()) {
        printf("robot rightLeg device not available.\n");
        trunkDevice.close();
        yarp::os::Network::fini();
        return false;    }
    if (!trunkDevice.view(trunkIEncoders) ) { // connecting our device with "IEncoders" interface
        printf("[warning] Problems acquiring trunkIEncoders interface\n");
        return false;
    } else printf("[success] Acquired trunkIEncoders interface\n");
    if (!trunkDevice.view(trunkIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring trunkIControlMode2 interface\n");
        return false;
    } else printf("[success] Acquired trunkIControlMode2 interface\n");
    if (!trunkDevice.view(trunkIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring trunkIPositionControl2 interface\n");
        return false;
    } else printf("[success] Acquired trunkIPositionControl2 interface\n");
    if (!trunkDevice.view(trunkIVelocityControl2) ) { // connecting our device with "velocity control 2" interface
        printf("[warning] Problems acquiring trunkIVelocityControl2 interface\n");
        return false;
    } else printf("[success] Acquired trunkIVelocityControl2 interface\n");
    if (!trunkDevice.view(trunkITorqueControl) ) { // connecting our device with "torque control 2" interface
        printf("[warning] Problems acquiring trunkITorqueControl2 interface\n");
        return false;
    } else printf("[success] Acquired trunkITorqueControl2 interface\n");
    /** **************************************************************************************
     * ******************************************************************************** **/


    // ------ LEFT LEG DEV -------
    yarp::os::Property leftLegOptions;
    leftLegOptions.put("device","remote_controlboard");
    leftLegOptions.put("remote",robot+"/leftLeg");
    leftLegOptions.put("local",waiterStr+"/teo/leftLeg");
    leftLegDevice.open(leftLegOptions);
    if(!leftLegDevice.isValid()) {
        printf("robot leftLeg device not available.\n");
        leftLegDevice.close();
        yarp::os::Network::fini();
        return false;    }
    if (!leftLegDevice.view(leftLegIEncoders) ) { // connecting our device with "IEncoders" interface
        printf("[warning] Problems acquiring leftLegIEncoders interface\n");
        return false;
    } else printf("[success] Acquired leftLegIEncoders interface\n");
    if (!leftLegDevice.view(leftLegIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring leftLegIControlMode2 interface\n");
        return false;
    } else printf("[success] Acquired leftLegIControlMode2 interface\n");

/*    if (!leftLegDevice.view(leftLegIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring leftLegIPositionControl2 interface\n");
        return false;
    } else printf("[success] Acquired leftLegIPositionControl2 interface\n");
    if (!leftLegDevice.view(leftLegIVelocityControl2) ) { // connecting our device with "velocity control 2" interface
        printf("[warning] Problems acquiring leftLegIVelocityControl2 interface\n");
        return false;
    } else printf("[success] Acquired leftLegIVelocityControl2 interface\n");*/
    if (!leftLegDevice.view(leftLegITorqueControl) ) { // connecting our device with "torque control 2" interface
        printf("[warning] Problems acquiring leftLegITorqueControl2 interface\n");
        return false;
    } else printf("[success] Acquired leftLegITorqueControl2 interface\n");
    /** **************************************************************************************
     * ******************************************************************************** **/


    // ------ RIGHT LEG DEV -------
    yarp::os::Property rightLegOptions;
    rightLegOptions.put("device","remote_controlboard");
    rightLegOptions.put("remote","/teo/rightLeg");
    rightLegOptions.put("local",waiterStr+robot+"/rightLeg");
    rightLegDevice.open(rightLegOptions);
    if(!rightLegDevice.isValid()) {
        printf("robot rightLeg device not available.\n");
        rightLegDevice.close();
        yarp::os::Network::fini();
        return false;    }
    if (!rightLegDevice.view(rightLegIEncoders) ) { // connecting our device with "IEncoders" interface
        printf("[warning] Problems acquiring rightLegIEncoders interface\n");
        return false;
    } else printf("[success] Acquired rightLegIEncoders interface\n");
    if (!rightLegDevice.view(rightLegIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring rightLegIControlMode2 interface\n");
        return false;
    } else printf("[success] Acquired rightLegIControlMode2 interface\n");
/*    if (!rightLegDevice.view(rightLegIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring rightLegIPositionControl2 interface\n");
        return false;
    } else printf("[success] Acquired rightLegIPositionControl2 interface\n");
    if (!rightLegDevice.view(rightLegIVelocityControl2) ) { // connecting our device with "velocity control 2" interface
        printf("[warning] Problems acquiring rightLegIVelocityControl2 interface\n");
        return false;
    } else printf("[success] Acquired rightLegIVelocityControl2 interface\n");*/
    if (!rightLegDevice.view(rightLegITorqueControl) ) { // connecting our device with "torque control 2" interface
        printf("[warning] Problems acquiring rightLegITorqueControl2 interface\n");
        return false;
    } else printf("[success] Acquired rightLegITorqueControl2 interface\n");
    /** **************************************************************************************
     * ******************************************************************************** **/


    // ----- SET POSITION CONTROL MODE -----
/*    trunkIPositionControl2->getAxes(&numTrunkJoints);
    std::vector<int> trunkControlModes(numTrunkJoints,VOCAB_CM_POSITION);
    if(! trunkIControlMode2->setControlModes( trunkControlModes.data() )){
        printf("[warning] Problems setting position control mode of: trunk\n");
        return false;
    }
    leftLegIPositionControl2->getAxes(&numLeftLegJoints);
    std::vector<int> leftLegControlModes(numLeftLegJoints,VOCAB_CM_POSITION);
    if(! leftLegIControlMode2->setControlModes( leftLegControlModes.data() )){
        printf("[warning] Problems setting position control mode of: left-Leg\n");
        return false;
    }
    rightLegIPositionControl2->getAxes(&numRightLegJoints);
    std::vector<int> rightLegControlModes(numRightLegJoints,VOCAB_CM_POSITION);
    if(! rightLegIControlMode2->setControlModes(rightLegControlModes.data())){
        printf("[warning] Problems setting position control mode of: right-Leg\n");
        return false;
    }*/

    // ----- SET VELOCITY CONTROL MODE -----
/*    trunkIVelocityControl2->getAxes(&numTrunkJoints);
    std::vector<int> trunkControlModes(numTrunkJoints,VOCAB_CM_VELOCITY);
    if(! trunkIControlMode2->setControlModes( trunkControlModes.data() )){
        printf("[warning] Problems setting velocity control mode of: trunk\n");
        return false;
    }
    leftLegIVelocityControl2->getAxes(&numLeftLegJoints);
    std::vector<int> leftLegControlModes(numLeftLegJoints,VOCAB_CM_VELOCITY);
    if(! leftLegIControlMode2->setControlModes( leftLegControlModes.data() )){
        printf("[warning] Problems setting velocity control mode of: left-Leg\n");
        return false;
    }
    rightLegIVelocityControl2->getAxes(&numRightLegJoints);
    std::vector<int> rightLegControlModes(numRightLegJoints,VOCAB_CM_VELOCITY);
    if(! rightLegIControlMode2->setControlModes(rightLegControlModes.data())){
        printf("[warning] Problems setting velocity control mode of: right-Leg\n");
        return false;
    }*/

    // ----- SET TORQUE CONTROL MODE -----
    trunkITorqueControl->getAxes(&numTrunkJoints);
/*    std::vector<int> trunkControlModes(numTrunkJoints,VOCAB_CM_TORQUE);
    if(! trunkIControlMode2->setControlModes( trunkControlModes.data() )){
        printf("[warning] Problems setting torque control mode of: trunk\n");
        return false;
    } else printf("[success] Set torque control mode of: trunk\n");*/
    leftLegITorqueControl->getAxes(&numLeftLegJoints);
/*    std::vector<int> leftLegControlModes(numLeftLegJoints,VOCAB_CM_TORQUE);
    if(! leftLegIControlMode2->setControlModes( leftLegControlModes.data() )){
        printf("[warning] Problems setting torque control mode of: left-Leg\n");
        return false;
    } else printf("[success] Set torque control mode of: left-Leg\n");*/
    rightLegITorqueControl->getAxes(&numRightLegJoints);
/*    std::vector<int> rightLegControlModes(numRightLegJoints,VOCAB_CM_TORQUE);
    if(! rightLegIControlMode2->setControlModes(rightLegControlModes.data())){
        printf("[warning] Problems setting torque control mode of: right-Leg\n");
        return false;
    } else printf("[success] Set torque control mode of: right-Leg\n");*/
    Time::delay(1);
    /** **************************************************************************************
     * ******************************************************************************** **/


    // ----- TRUNK KDL SOLVER -----
    if( ! trunkDevice.view(trunkIControlLimits) ) {
        printf("Could not view iControlLimits in leftLegDevice\n");
        return false;    }

    printf("---- Joint limits of left-leg ---- \n");
    yarp::os::Bottle qtMin, qtMax;    //  Getting the limits of each joint
        for(unsigned int joint=0;joint<numTrunkJoints;joint++)        {
            double min, max;
            trunkIControlLimits->getLimits(joint,&min,&max);
            qtMin.addDouble(min);
            qtMax.addDouble(max);
            printf("Joint %d limits: [%f,%f]\n",joint,min,max);        }

    yarp::os::Property trunkSolverOptions;
    trunkSolverOptions.fromConfigFile("/usr/local/share/WBPCwaiter/contexts/kinematics/trunkKinematics-waiter.ini");
    trunkSolverOptions.put("device", "KdlSolver");
    trunkSolverOptions.put("mins", yarp::os::Value::makeList(qtMin.toString().c_str()));
    trunkSolverOptions.put("maxs", yarp::os::Value::makeList(qtMax.toString().c_str()));
    trunkSolverDevice.open(trunkSolverOptions);
    if( ! trunkDevice.isValid() )    {
        CD_ERROR("[ERROR] KDLSolver solver device for left-arm is not valid \n");
        return false;
    }
    if( ! trunkDevice.view(trunkICartesianSolver) )    {
        CD_ERROR("[ERROR] Could not view iCartesianSolver in KDLSolver\n");
        return false;
    } else printf("[success] Acquired leftLegICartesianSolver interface\n");
    /** **************************************************************************************
     * ******************************************************************************** **/


    // ----- LEFT LEG KDL SOLVER -----
    if( ! leftLegDevice.view(leftLegIControlLimits) ) {
        printf("Could not view iControlLimits in leftLegDevice\n");
        return false;    }

    printf("---- Joint limits of left-leg ---- \n");
    yarp::os::Bottle qlMin, qlMax;    //  Getting the limits of each joint
        for(unsigned int joint=0;joint<numLeftLegJoints;joint++)        {
            double min, max;
            leftLegIControlLimits->getLimits(joint,&min,&max);
            qlMin.addDouble(min);
            qlMax.addDouble(max);
            printf("Joint %d limits: [%f,%f]\n",joint,min,max);        }

    yarp::os::Property leftLegSolverOptions;
    leftLegSolverOptions.fromConfigFile("/usr/local/share/WBPCwaiter/contexts/kinematics/leftLegKinematics-waiter.ini");
    leftLegSolverOptions.put("device", "KdlSolver");
    leftLegSolverOptions.put("mins", yarp::os::Value::makeList(qlMin.toString().c_str()));
    leftLegSolverOptions.put("maxs", yarp::os::Value::makeList(qlMax.toString().c_str()));
    leftLegSolverDevice.open(leftLegSolverOptions);
    if( ! leftLegSolverDevice.isValid() )    {
        CD_ERROR("[ERROR] KDLSolver solver device for left-arm is not valid \n");
        return false;
    }
    if( ! leftLegSolverDevice.view(leftLegICartesianSolver) )    {
        CD_ERROR("[ERROR] Could not view iCartesianSolver in KDLSolver\n");
        return false;
    } else printf("[success] Acquired leftLegICartesianSolver interface\n");
    /** **************************************************************************************
     * ******************************************************************************** **/
    // ----- RIGHT LEG KDL SOLVER -----
    if( ! rightLegDevice.view(rightLegIControlLimits) ) {
        printf("Could not view iControlLimits in rightLegDevice\n");
        return false;
    }

    printf("---- Joint limits of right-Leg ----\n");
    yarp::os::Bottle qrMin, qrMax;     //  Getting the limits of each joint
        for(unsigned int joint=0;joint<numRightLegJoints;joint++)
        {
            double min, max;
            rightLegIControlLimits->getLimits(joint,&min,&max);
            qrMin.addDouble(min);
            qrMax.addDouble(max);
            printf("Joint %d limits: [%f,%f]\n",joint,min,max);
        }

    yarp::os::Property rightLegSolverOptions;
    rightLegSolverOptions.fromConfigFile("/usr/local/share/WBPCwaiter/contexts/kinematics/rightLegKinematics-waiter.ini");
    rightLegSolverOptions.put("device","KdlSolver");
    rightLegSolverOptions.put("mins", yarp::os::Value::makeList(qrMin.toString().c_str()));
    rightLegSolverOptions.put("maxs", yarp::os::Value::makeList(qrMax.toString().c_str()));
    rightLegSolverDevice.open(rightLegSolverOptions);
    if( ! rightLegSolverDevice.isValid() )    {
        printf("[ERROR] KDLSolver solver device for right-Leg is not valid \n");
        return false;
    }
    if( ! rightLegSolverDevice.view(rightLegICartesianSolver) )    {
        printf("[ERROR] Could not view iCartesianSolver in KDLSolver \n");
        return false;
    } else printf("[success] Acquired rightLegICartesianSolver interface\n");
    /** **************************************************************************************
     * ******************************************************************************** **/


    yarp::os::Time::delay(1);

    //-- Conection between TestCap53 & ThreadImpl
    //threadImpl.setNumJoints(numLeftArmJoints);
    threadImpl.setIEncodersControl(rightLegIEncoders,leftLegIEncoders,trunkIEncoders);
    threadImpl.setIPositionControl2(rightLegIPositionControl2,leftLegIPositionControl2,trunkIPositionControl2);
    threadImpl.setIVelocityControl2(rightLegIVelocityControl2,leftLegIVelocityControl2,trunkIVelocityControl2); // no se utiliza de momento
    threadImpl.setITorqueControl(rightLegITorqueControl,leftLegITorqueControl,trunkITorqueControl); // no se utiliza de momento
    threadImpl.setICartesianSolver(rightLegICartesianSolver,leftLegICartesianSolver,trunkICartesianSolver);
    threadImpl.setInputPorts(&portImu,&portft0,&portft1,&portft2,&portft3);

    threadImpl.start();

    return true;
}

/************************************************************************/
double TestCap53a::getPeriod() {
    return 4.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool TestCap53a::updateModule() {
    printf("TestCap53a alive...\n");
    return true;
}

/************************************************************************/
bool TestCap53a::interruptModule() {
    printf("Test53a closing...\n");

    threadImpl.stop();

    trunkDevice.close();
    rightLegDevice.close();
    leftLegDevice.close();

    return true;
}

/************************************************************************/
} // namespace roboticslab
