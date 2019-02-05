// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
el test 52 está asociado para el HUMANOID 2018 en el que el topic del paper es el modelo de
control de la botella basado en rampas variables usando el sensor de fuerza/par (CH3).

el test 52 consiste en observar y comprobar si el ZMP se calcula correctamente en funcion del
angulo de inclinacion de la bandeja. Este test se debe dividir a mano en otros tres, en el que
se medirá el ZMP, tanto en el plano frontal (0,1,0), como en el plano sagital (1,0,0), como en
un plano que sea combinacion de los dos anteriores (1,1,0).

Unicamente se utiliza el brazo izquierdo, luego solo es necesario crear el device de esta
extremidad y el solver asociado a esta.
*/

#include "TestCap52.hpp"

namespace roboticslab
{

/************************************************************************/
bool TestCap52::configure(ResourceFinder &rf) {

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
    std::string waiterStr("/objectBal");

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


    // ------ LEFT ARM DEV -------
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
    } else printf("[success] Acquired leftArmIPositionControl2 interface\n");
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


    // ------ TRUNK DEV -------
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
    } else printf("[success] Acquired trunkIPositionControl2 interface\n");
    /** **************************************************************************************
     * ******************************************************************************** **/


    /* // ------ LEFT LEG DEV -------
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
    } else printf("[success] Acquired leftLegIPositionControl2 interface\n");*/
    /** **************************************************************************************
     * ******************************************************************************** **/


    /* // ------ RIGHT LEG DEV -------
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
    } else printf("[success] Acquired rightLegIPositionControl2 interface\n");  */
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
    leftArmIPositionControl2->getAxes(&numLeftArmJoints);
    std::vector<int> leftArmControlModes(numLeftArmJoints,VOCAB_CM_POSITION);
    if(! leftArmIControlMode2->setControlModes( leftArmControlModes.data() )){
        printf("[warning] Problems setting position control mode of: left-arm\n");
        return false;    }
/*    rightArmIPositionControl2->getAxes(&numRightArmJoints);
    std::vector<int> rightArmControlModes(numRightArmJoints,VOCAB_CM_POSITION);
    if(! rightArmIControlMode2->setControlModes(rightArmControlModes.data())){
        printf("[warning] Problems setting position control mode of: right-arm\n");
        return false;
    }*/
    // conf lower body
    trunkIPositionControl2->getAxes(&numTrunkJoints);
    std::vector<int> trunkControlModes(numTrunkJoints,VOCAB_CM_POSITION);
    if(! trunkIControlMode2->setControlModes( trunkControlModes.data() )){
        printf("[warning] Problems setting position control mode of: trunk\n");
        return false;    }
/*    leftLegIPositionControl2->getAxes(&numLeftLegJoints);
    std::vector<int> leftLegControlModes(numLeftLegJoints,VOCAB_CM_POSITION);
    if(! leftLegIControlMode2->setControlModes( leftLegControlModes.data() )){
        printf("[warning] Problems setting position control mode of: left-Leg\n");
        return false;
    }*/
/*    rightLegIPositionControl2->getAxes(&numRightLegJoints);
    std::vector<int> rightLegControlModes(numRightLegJoints,VOCAB_CM_POSITION);
    if(! rightLegIControlMode2->setControlModes(rightLegControlModes.data())){
        printf("[warning] Problems setting position control mode of: right-Leg\n");
        return false;
    }*/
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
    /** **************************************************************************************
     * ******************************************************************************** **/
    // conf left-arm kdl
    if( ! leftArmDevice.view(leftArmIControlLimits) ) {
        printf("Could not view iControlLimits in leftArmDevice\n");
        return false;    }

    printf("---- Joint limits of left-arm ---- \n");
    yarp::os::Bottle qlMin, qlMax;    //  Getting the limits of each joint
        for(unsigned int joint=0;joint<numLeftArmJoints;joint++)        {
            double min, max;
            leftArmIControlLimits->getLimits(joint,&min,&max);
            qlMin.addDouble(min);
            qlMax.addDouble(max);
            printf("Joint %d limits: [%f,%f]\n",joint,min,max);        }

    yarp::os::Property leftArmSolverOptions;
    leftArmSolverOptions.fromConfigFile("/usr/local/share/manip-waiter/contexts/kinematics/leftArmKinematics-waiter.ini");
    leftArmSolverOptions.put("device", "KdlSolver");
    leftArmSolverOptions.put("mins", yarp::os::Value::makeList(qlMin.toString().c_str()));
    leftArmSolverOptions.put("maxs", yarp::os::Value::makeList(qlMax.toString().c_str()));
    leftArmSolverDevice.open(leftArmSolverOptions);
    if( ! leftArmSolverDevice.isValid() )    {
        CD_ERROR("[ERROR] KDLSolver solver device for left-arm is not valid \n");
        return false;
    }
    if( ! leftArmSolverDevice.view(leftArmICartesianSolver) )    {
        CD_ERROR("[ERROR] Could not view iCartesianSolver in KDLSolver\n");
        return false;
    } else printf("[success] Acquired leftArmICartesianSolver interface\n");
    /** **************************************************************************************
     * ******************************************************************************** **/


    // ----- FT2 SENSOR DEVICE -----
    yarp::os::Property ft2SensorOptions;
    ft2SensorOptions.put("device","analogsensorclient");
    ft2SensorOptions.put("remote","/jr3/ch2:o");
    ft2SensorOptions.put("local",waiterStr+"/jr3/ch2:i");

    ft2SensorDevice.open(ft2SensorOptions); // FT 2 Sensor Device
    if(!ft2SensorDevice.isValid()) {
      printf("Device not available.\n");
      ft2SensorDevice.close();
      yarp::os::Network::fini();
      return false;    }

    if ( ! ft2SensorDevice.view(iFT2AnalogSensor) )    {
        std::printf("[error] Problems acquiring interface\n");
        return false;
    } else printf("[success] acquired interface\n");

    yarp::os::Time::delay(1);   // The following delay should avoid 0 channels and bad read

    int channelsFT2 = iFT2AnalogSensor->getChannels();
    printf("channels: %d\n", channelsFT2);
    /** **************************************************************************************
     * ******************************************************************************** **/


    // ----- FT3 SENSOR DEVICE -----
    yarp::os::Property ft3SensorOptions;
    ft3SensorOptions.put("device","analogsensorclient");
    ft3SensorOptions.put("remote","/jr3/ch3:o");
    ft3SensorOptions.put("local",waiterStr+"/jr3/ch3:i");

    ft3SensorDevice.open(ft3SensorOptions); // FT 3 Sensor Device
    if(!ft3SensorDevice.isValid()) {
      printf("Device not available.\n");
      ft3SensorDevice.close();
      yarp::os::Network::fini();
      return false;    }

    if ( ! ft3SensorDevice.view(iFT3AnalogSensor) )    {
        printf("[error] Problems acquiring interface\n");
        return false;
    } else printf("[success] acquired interface\n");

    yarp::os::Time::delay(1);   // The following delay should avoid 0 channels and bad read

    int channelsFT3 = iFT3AnalogSensor->getChannels();
    printf("channels: %d\n", channelsFT3);
    /** **************************************************************************************
     * ******************************************************************************** **/


/*    //-- OPEN YARP PORTS
    portImu.open("/waiter/inertial:i");
    portft0.open("/waiter/jr3ch0:i");
    portft1.open("/waiter/jr3ch1:i");
    portft2.open("/waiter/jr3ch2:i");
    portft3.open("/waiter/jr3ch3:i");
    yarp::os::Time::delay(0.5);

    //-- CONNECTIONS PORTS
    // ft right foot
    Network::connect("/jr3/ch0:o","/waiter/jr3ch0:i");
    if ( NetworkBase::isConnected("/jr3/ch0:o","/waiter/jr3ch0:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /waiter/jr3ch0:i." << endl;
    } else cout << "[success] Connected to /jr3ch0:i." << endl;
    yarp::os::Time::delay(0.5);
    // ft left foot
    Network::connect("/jr3/ch1:o","/waiter/jr3ch1:i");
    if ( NetworkBase::isConnected("/jr3/ch1:o","/waiter/jr3ch1:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /waiter/jr3ch1:i." << endl;
    } else cout << "[success] Connected to /jr3ch1:i." << endl;
    yarp::os::Time::delay(0.5);
    // ft right hand
    Network::connect("/jr3/ch2:o","/waiter/jr3ch2:i");
    if ( NetworkBase::isConnected("/jr3/ch2:o","/waiter/jr3ch2:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /waiter/jr3ch2:i." << endl;
    } else cout << "[success] Connected to /jr3ch2:i." << endl;
    yarp::os::Time::delay(0.5);
    // ft left hand
    Network::connect("/jr3/ch3:o","/waiter/jr3ch3:i");
    if ( NetworkBase::isConnected("/jr3/ch3:o","/waiter/jr3ch3:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /waiter/jr3ch3:i." << endl;
    } else cout << "[success] Connected to /jr3ch3:i." << endl;
    yarp::os::Time::delay(0.5);
    // imu trunk
    Network::connect("/inertial", "/waiter/inertial:i");
    if ( NetworkBase::isConnected("/inertial", "/waiter/inertial:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /waiter/inertial:i." << endl;
    } else cout << "[success] Connected to IMU." << endl;
    yarp::os::Time::delay(0.5);*/
    /** **************************************************************************************
     * ******************************************************************************** **/


    //-- Set home waiter poss & Initial SPEED-ACC
    configInitPosition(25,25);
    double leftArmInitPoss[7] = {-30,0,0,-90,0,30,0};
    double trunkInitPoss[2] = {0,-2.5};
    std::vector<double> leftArm(&leftArmInitPoss[0], &leftArmInitPoss[0]+7);
    std::vector<double> trunk(&trunkInitPoss[0], &trunkInitPoss[0]+2);
    moveJointsInitPosition(trunk, leftArm);

    yarp::os::Time::delay(5); // checkMotionDone() doesn't work very well

    /** **************************************************************************************
     * ******************************************************************************** **/


    //-- Calibrating the sensor
    int ret = iFT3AnalogSensor->calibrateChannel(3);
    // ret value is not checked due to a bug in yarp(2.3.70/2.3.72.1)
    /*if (ret != yarp::dev::IAnalogSensor::AS_OK) {
        printf("[ERROR] Calibrating Channel ...\n");
        return false;    }
    else printf("[OK] All channels reseted\n");*/
    /** **************************************************************************************
     * ******************************************************************************** **/


    //-- Conection between TestCap52 & ThreadImpl
    //threadImpl.setNumJoints(numLeftArmJoints); // not used
    threadImpl.setIEncodersControl(leftArmIEncoders);
    threadImpl.setLeftArmIPositionControl2(leftArmIPositionControl2);
    threadImpl.setTrunkIPositionControl2(trunkIPositionControl2); // not used
    //threadImpl.setIVelocityControl2(leftArmIVelocityControl2); // not used
    threadImpl.setICartesianSolver(leftArmICartesianSolver);
    threadImpl.setInputPorts(&portImu,&portft0,&portft1,&portft2,&portft3);
    threadImpl.setIAnalogSensor(iFT2AnalogSensor,iFT3AnalogSensor);
    //yarp::dev::IAnalogSensor *iFT3AnalogSensor;

    threadImpl.start();

    return true;
}

/************************************************************************/
bool TestCap52::configInitPosition(double speed, double acc){

    // -- Speed and acceleration for 7 joints
    std::vector<double> armSpeeds(7, speed); // 7,30.0
    std::vector<double> armAccelerations(7, acc); // 7,30.0

    // -- Configuring Device to Position Mode

    if (!leftArmDevice.view(leftArmIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        CD_ERROR("Problems acquiring leftArmIPositionControl2 interface\n");
        return false;
    } else CD_INFO_NO_HEADER("Acquired leftArmIPositionControl2 interface\n");

    std::vector<int> leftArmControlModes(numLeftArmJoints,VOCAB_CM_POSITION);
    if(! leftArmIControlMode2->setControlModes( leftArmControlModes.data() )){
        CD_ERROR("Problems setting position control mode of: left-arm\n");
        return false;    }

    // -- Configuring speed and acceleration
    if(!leftArmIPositionControl2->setRefSpeeds(armSpeeds.data())){
        CD_ERROR("Problems setting reference speed on left-arm joints.\n");
        return false;    }
    if(!leftArmIPositionControl2->setRefAccelerations(armAccelerations.data())){
        CD_ERROR("Problems setting reference acceleration on left-arm joints.\n");
        return false;    }

    return true;
}

/************************************************************************/
bool TestCap52::moveJointsInitPosition(vector<double> &trunk, vector<double>& leftArm)
{
    bool doneTrunk = false;
    bool doneLeftArm = false;

    // -- moving to position
    if(!trunkIPositionControl2->positionMove( trunk.data() )){
        CD_ERROR("[Error: positionMove] Problems setting new reference point for trunk axes.\n");
        return false;    }
    if(!leftArmIPositionControl2->positionMove( leftArm.data() )){
        CD_ERROR("[Error: positionMove] Problems setting new reference point for left-arm axes.\n");
        return false;    }

    // -- checking movement done...
    while(!doneTrunk)    {
        yarp::os::Time::delay(0.1);
        trunkIPositionControl2->checkMotionDone(&doneTrunk);    }
    while(!doneLeftArm)    {
        yarp::os::Time::delay(0.1);
        leftArmIPositionControl2->checkMotionDone(&doneLeftArm);    }

    return true;
}

/************************************************************************/
double TestCap52::getPeriod() {
    return 4.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool TestCap52::updateModule() {
    printf("TestCap52 alive...\n");
    return true;
}

/************************************************************************/
bool TestCap52::interruptModule() {
    printf("TestCap52 closing...\n");

    threadImpl.stop();

    //rightArmSolverDevice.close(); // not used
    leftArmSolverDevice.close();

    //headDevice.close(); // not used
    //rightArmDevice.close(); // not used
    leftArmDevice.close();

    trunkDevice.close(); // not used
    //rightLegDevice.close(); // not used
    //leftLegDevice.close(); // not used

    ft2SensorDevice.close();
    ft3SensorDevice.close();

    return true;
}

/************************************************************************/
} // namespace roboticslab
