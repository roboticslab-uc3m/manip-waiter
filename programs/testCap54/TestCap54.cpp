// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**

the idea is make the first step for a body balance control for the waiter.
this TestCap54 start with the verification of previous TestCap53. we generate
a simple control:
    ->  focus on push recovery test and ankle strategy.
    ->  with the validate DLIPM.

we include IMU sensor only for measure the external disturbance.

there can be a second part. we can extend this test with hip strategy or using:
    ->  "velocity" or "PositionDirect" mode control

**/

#include "TestCap54.hpp"

namespace roboticslab
{

/************************************************************************/
bool TestCap54::configure(ResourceFinder &rf) {

    std::string robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"name of /robot to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("testCap54 options:\n");
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

/*    // ------ HEAD DEV -------
    yarp::os::Property headOptions;
    headOptions.put("device","remote_controlboard");
    headOptions.put("remote",robot+"/head");
    headOptions.put("local",waiterStr+"/teo/head");
    headDevice.open(headOptions);
    if(!headDevice.isValid()) {
      printf("robot head device not available.\n");
      headDevice.close();
      yarp::os::Network::fini();
      return false;    }
    if (!headDevice.view(headIEncoders) ) { // connecting our device with "IEncoders" interface
        printf("[warning] Problems acquiring headIEncoders interface\n");
        return false;
    } else printf("[success] Acquired headIEncoders interface\n");
    if (!headDevice.view(headIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring headIControlMode2 interface\n");
        return false;
    } else printf("[success] Acquired headIControlMode2 interface\n");

    if (!headDevice.view(headIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring headIPositionControl2 interface\n");
        return false;
    } else printf("[success] Acquired headIPositionControl2 interface\n");*/
    /** **************************************************************************************
     * ******************************************************************************** **/

/*    // ------ LEFT ARM DEV -------
    yarp::os::Property leftArmOptions;
    leftArmOptions.put("device","remote_controlboard");
    leftArmOptions.put("remote",robot+"/leftArm");
    leftArmOptions.put("local",waiterStr+"/teo/leftArm");
    leftArmDevice.open(leftArmOptions);
    if(!leftArmDevice.isValid()) {
      printf("robot leftArm device not available.\n");
      leftArmDevice.close();
      yarp::os::Network::fini();
      return false;    }
    if (!leftArmDevice.view(leftArmIEncoders) ) { // connecting our device with "IEncoders" interface
        printf("[warning] Problems acquiring leftArmIEncoders interface\n");
        return false;
    } else printf("[success] Acquired leftArmIEncoders interface\n");
    if (!leftArmDevice.view(leftArmIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring leftArmIControlMode2 interface\n");
        return false;
    } else printf("[success] Acquired leftArmIControlMode2 interface\n");
    if (!leftArmDevice.view(leftArmIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring leftArmIPositionControl2 interface\n");
        return false;
    } else printf("[success] Acquired leftArmIPositionControl2 interface\n");*/
    /** **************************************************************************************
     * ******************************************************************************** **/

/*    // ------ RIGHT ARM DEV -------
    yarp::os::Property rightArmOptions;
    rightArmOptions.put("device","remote_controlboard");
    rightArmOptions.put("remote","/teo/rightArm");
    rightArmOptions.put("local",waiterStr+"/teo/rightArm");
    rightArmDevice.open(rightArmOptions);
    if(!rightArmDevice.isValid()) {
        printf("robot rightArm device not available.\n");
        rightArmDevice.close();
        yarp::os::Network::fini();
        return false;    }
    if (!rightArmDevice.view(rightArmIEncoders) ) { // connecting our device with "IEncoders" interface
        printf("[warning] Problems acquiring rightArmIEncoders interface\n");
        return false;
    } else printf("[success] Acquired rightArmIEncoders interface\n");
    if (!rightArmDevice.view(rightArmIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        printf("[warning] Problems acquiring rightArmIControlMode2 interface\n");
        return false;
    } else printf("[success] Acquired rightArmIControlMode2 interface\n");
    if (!rightArmDevice.view(rightArmIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring rightArmIPositionControl2 interface\n");
        return false;
    } else printf("[success] Acquired rightArmIPositionControl2 interface\n");*/
    /** **************************************************************************************
     * ******************************************************************************** **/

/*    // ------ TRUNK DEV -------
    yarp::os::Property trunkOptions;
    trunkOptions.put("device","remote_controlboard");
    trunkOptions.put("remote",robot+"/trunk");
    trunkOptions.put("local",waiterStr+"/teo/trunk");
    trunkDevice.open(trunkOptions);
    if(!trunkDevice.isValid()) {
        printf("robot trunk device not available.\n");
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
    } else printf("[success] Acquired trunkIPositionControl2 interface\n");*/
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

    if (!leftLegDevice.view(leftLegIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring leftLegIPositionControl2 interface\n");
        return false;
    } else printf("[success] Acquired leftLegIPositionControl2 interface\n");
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
    if (!rightLegDevice.view(rightLegIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        printf("[warning] Problems acquiring rightLegIPositionControl2 interface\n");
        return false;
    } else printf("[success] Acquired rightLegIPositionControl2 interface\n");
    /** **************************************************************************************
     * ******************************************************************************** **/

    // ----- SET CONTROL MODES -----
    // conf upper body
/*    headIPositionControl2->getAxes(&numHeadJoints);
    std::vector<int> headControlModes(numHeadJoints,VOCAB_CM_POSITION);
    if(! headIControlMode2->setControlModes( headControlModes.data() )){
        printf("[warning] Problems setting position control mode of: head\n");
        return false;
    }*/
/*    leftArmIPositionControl2->getAxes(&numLeftArmJoints);
    std::vector<int> leftArmControlModes(numLeftArmJoints,VOCAB_CM_POSITION);
    if(! leftArmIControlMode2->setControlModes( leftArmControlModes.data() )){
        printf("[warning] Problems setting position control mode of: left-arm\n");
        return false;
    }*/
/*    rightArmIPositionControl2->getAxes(&numRightArmJoints);
    std::vector<int> rightArmControlModes(numRightArmJoints,VOCAB_CM_POSITION);
    if(! rightArmIControlMode2->setControlModes(rightArmControlModes.data())){
        printf("[warning] Problems setting position control mode of: right-arm\n");
        return false;
    }*/
    // conf lower body
/*    trunkIPositionControl2->getAxes(&numTrunkJoints);
    std::vector<int> trunkControlModes(numTrunkJoints,VOCAB_CM_POSITION);
    if(! trunkIControlMode2->setControlModes( trunkControlModes.data() )){
        printf("[warning] Problems setting position control mode of: trunk\n");
        return false;
    }*/
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
    }
    /** **************************************************************************************
     * ******************************************************************************** **/

    // ----- KDL SOLVER -----
/*    // conf right-arm kdl
    if( ! rightArmDevice.view(rightArmIControlLimits) ) {
        printf("Could not view iControlLimits in rightArmDevice\n");
        return false;
    }

    printf("---- Joint limits of right-arm ----\n");
    yarp::os::Bottle qrMin, qrMax;     //  Getting the limits of each joint
        for(unsigned int joint=0;joint<numRightArmJoints;joint++)
        {
            double min, max;
            rightArmIControlLimits->getLimits(joint,&min,&max);
            qrMin.addDouble(min);
            qrMax.addDouble(max);
            printf("Joint %d limits: [%f,%f]\n",joint,min,max);
        }

    yarp::os::Property rightArmSolverOptions;
    rightArmSolverOptions.fromConfigFile("/usr/local/share/teo-configuration-files/contexts/kinematics/rightArmKinematics.ini");
    rightArmSolverOptions.put("device","KdlSolver");
    rightArmSolverOptions.put("mins", yarp::os::Value::makeList(qrMin.toString().c_str()));
    rightArmSolverOptions.put("maxs", yarp::os::Value::makeList(qrMax.toString().c_str()));
    rightArmSolverDevice.open(rightArmSolverOptions);
    if( ! rightArmSolverDevice.isValid() )    {
        printf("[ERROR] KDLSolver solver device for right-arm is not valid \n");
        return false;
    }
    if( ! rightArmSolverDevice.view(rightArmICartesianSolver) )    {
        printf("[ERROR] Could not view iCartesianSolver in KDLSolver \n");
        return false;
    } else printf("[success] Acquired rightArmICartesianSolver interface\n");*/

/*    // conf left-arm kdl
    if( ! leftArmDevice.view(leftArmIControlLimits) ) {
        printf("Could not view iControlLimits in leftArmDevice\n");
        return false;
    }

    printf("---- Joint limits of left-arm ---- \n");
    yarp::os::Bottle qlMin, qlMax;    //  Getting the limits of each joint
        for(unsigned int joint=0;joint<numLeftArmJoints;joint++)        {
            double min, max;
            leftArmIControlLimits->getLimits(joint,&min,&max);
            qlMin.addDouble(min);
            qlMax.addDouble(max);
            printf("Joint %d limits: [%f,%f]\n",joint,min,max);
        }

    yarp::os::Property leftArmSolverOptions;
    leftArmSolverOptions.fromConfigFile("/usr/local/share/manip-waiter/contexts/kinematics/leftArmKinematics-waiter.ini");
    leftArmSolverOptions.put("device", "KdlSolver");
    leftArmSolverOptions.put("mins", yarp::os::Value::makeList(qlMin.toString().c_str()));
    leftArmSolverOptions.put("maxs", yarp::os::Value::makeList(qlMax.toString().c_str()));
    leftArmSolverDevice.open(leftArmSolverOptions);
    if( ! leftArmSolverDevice.isValid() )    {
        printf("[ERROR] KDLSolver solver device for left-arm is not valid \n");
        return false;
    }
    if( ! leftArmSolverDevice.view(leftArmICartesianSolver) )    {
        printf("[ERROR] Could not view iCartesianSolver in KDLSolver\n");
        return false;
    } else printf("[success] Acquired leftArmICartesianSolver interface\n");*/
    /** **************************************************************************************
     * ******************************************************************************** **/


    // ----- FT0 SENSOR DEVICE -----
    yarp::os::Property ft0SensorOptions;
    ft0SensorOptions.put("device","analogsensorclient");
    ft0SensorOptions.put("remote","/jr3/ch0:o");
    ft0SensorOptions.put("local",waiterStr+"/jr3/ch0:i");

    ft0SensorDevice.open(ft0SensorOptions); // FT 0 Sensor Device
    if(!ft0SensorDevice.isValid()) {
      printf("Device not available.\n");
      ft0SensorDevice.close();
      yarp::os::Network::fini();
      return false;    }

    if ( ! ft0SensorDevice.view(iFT0AnalogSensor) )    {
        std::printf("[error] Problems acquiring interface\n");
        return false;
    } else printf("[success] acquired SENSOR FT 0 interface\n");

    yarp::os::Time::delay(1);   // The following delay should avoid 0 channels and bad read

    int channelsFT0 = iFT0AnalogSensor->getChannels();
    printf("channels: %d\n", channelsFT0);
    /** **************************************************************************************
     * ******************************************************************************** **/


    // ----- FT1 SENSOR DEVICE -----
    yarp::os::Property ft1SensorOptions;
    ft1SensorOptions.put("device","analogsensorclient");
    ft1SensorOptions.put("remote","/jr3/ch1:o");
    ft1SensorOptions.put("local",waiterStr+"/jr3/ch1:i");

    ft1SensorDevice.open(ft1SensorOptions); // FT 2 Sensor Device
    if(!ft1SensorDevice.isValid()) {
      printf("Device not available.\n");
      ft1SensorDevice.close();
      yarp::os::Network::fini();
      return false;    }

    if ( ! ft1SensorDevice.view(iFT1AnalogSensor) )    {
        std::printf("[error] Problems acquiring interface\n");
        return false;
    } else printf("[success] acquired SENSOR FT 1 interface\n");

    yarp::os::Time::delay(1);   // The following delay should avoid 0 channels and bad read

    int channelsFT1 = iFT1AnalogSensor->getChannels();
    printf("channels: %d\n", channelsFT1);
    /** **************************************************************************************
     * ******************************************************************************** **/


    yarp::os::Time::delay(1);

    //-- OPEN AND CONNECTIONS YARP PORTS
    portImu.open(waiterStr+"/inertial:i");
    yarp::os::Time::delay(0.5);

    // imu trunk
    Network::connect("/inertial", waiterStr+"/inertial:i");
    if ( NetworkBase::isConnected("/inertial",waiterStr+"/inertial:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /inertial:i." << endl;
    } else cout << "[success] Connected to IMU." << endl;
    yarp::os::Time::delay(0.5);
    /** **************************************************************************************
     * ******************************************************************************** **/

    //-- Conection between TestCap54 & ThreadImpl
    //threadImpl.setNumJoints(numLeftArmJoints);
    threadImpl.setIEncodersControl(rightLegIEncoders,leftLegIEncoders);
    threadImpl.setIPositionControl2(rightLegIPositionControl2,leftLegIPositionControl2);
    //threadImpl.setIVelocityControl2(rightLegIVelocityControl2,leftLegIVelocityControl2); // no se utiliza de momento
    threadImpl.setInputPorts(&portImu);
    threadImpl.setIAnalogSensor(iFT0AnalogSensor,iFT1AnalogSensor,iFT2AnalogSensor,iFT3AnalogSensor);

    threadImpl.start();

    return true;
}

/************************************************************************/
double TestCap54::getPeriod() {
    return 4.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool TestCap54::updateModule() {
    printf("TestCap51 alive...\n");
    return true;
}

/************************************************************************/
bool TestCap54::interruptModule() {
    printf("Test51 closing...\n");

    threadImpl.stop();

    rightArmSolverDevice.close();
    leftArmSolverDevice.close();

    headDevice.close();
    rightArmDevice.close();
    leftArmDevice.close();

    trunkDevice.close();
    rightLegDevice.close();
    leftLegDevice.close();

    return true;
}

/************************************************************************/
} // namespace roboticslab
