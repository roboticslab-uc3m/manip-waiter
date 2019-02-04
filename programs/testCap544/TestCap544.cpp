// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**

the idea is make the first step for a body balance control for the waiter.
but now , the mode control will be positionDirect. based on the 
teo-bimanipulation repo (raul santos), we have to include the interpolate 
generator for each time sample. 

the interpolate trayectory should change, if the output changes.

if in each loop, the trayectory must change, maybe the interpolate generator 
should use a small value of points to go faster.

**/

#include "TestCap544.hpp"

namespace roboticslab
{

/************************************************************************/
bool TestCap544::configure(ResourceFinder &rf) {

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

    // ------ LEFT LEG DEV -------
    yarp::os::Property leftLegOptions;
    leftLegOptions.put("device","remote_controlboard");
    leftLegOptions.put("remote",robot+"/leftLeg");
    leftLegOptions.put("local",waiterStr+"/teo/leftLeg");
    leftLegDevice.open(leftLegOptions);
    if(!leftLegDevice.isValid()) {
        CD_ERROR("robot leftLeg device not available.\n");
        leftLegDevice.close();
        yarp::os::Network::fini();
        return false;    }
    if (!leftLegDevice.view(leftLegIEncoders) ) { // connecting our device with "IEncoders" interface
        CD_ERROR("Problems acquiring leftLegIEncoders interface\n");
        return false;
    } else CD_INFO(" Acquired leftLegIEncoders interface\n");
    if (!leftLegDevice.view(leftLegIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        CD_ERROR("Problems acquiring leftLegIControlMode2 interface\n");
        return false;
    } else CD_INFO("Acquired leftLegIControlMode2 interface\n");

    if (!leftLegDevice.view(leftLegIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        CD_ERROR("Problems acquiring leftLegIPositionControl2 interface\n");
        return false;
    } else CD_INFO("Acquired leftLegIPositionControl2 interface\n");

    /** **************************************************************************************
     * ******************************************************************************** **/

    // ------ RIGHT LEG DEV -------
    yarp::os::Property rightLegOptions;
    rightLegOptions.put("device","remote_controlboard");
    rightLegOptions.put("remote",robot+"/rightLeg");
    rightLegOptions.put("local",waiterStr+robot+"/rightLeg");
    rightLegDevice.open(rightLegOptions);
    if(!rightLegDevice.isValid()) {
        CD_ERROR("robot rightLeg device not available.\n");
        rightLegDevice.close();
        yarp::os::Network::fini();
        return false;    }
    if (!rightLegDevice.view(rightLegIControlMode2) ) { // connecting our device with "control mode 2" interface, initializing which control mode we want (position)
        CD_ERROR("Problems acquiring rightLegIControlMode2 interface\n");
        return false;
    } else CD_INFO("Acquired rightLegIControlMode2 interface\n");
    if (!rightLegDevice.view(rightLegIPositionControl2) ) { // connecting our device with "position control 2" interface (configuring our device: speed, acceleration... and sending joint positions)
        CD_ERROR("Problems acquiring rightLegIPositionControl2 interface\n");
        return false;
    } else CD_INFO("Acquired rightLegIPositionControl2 interface\n");
    if (!rightLegDevice.view(rightLegIEncoders) ) { // connecting our device with "IEncoders" interface
        CD_ERROR("Problems acquiring rightLegIEncoders interface\n");
        return false;
    } else CD_INFO("Acquired rightLegIEncoders interface\n");

    /** **************************************************************************************
     * ******************************************************************************** **/

    // ----- SET CONTROL MODES -----
    leftLegIPositionControl2->getAxes(&numLeftLegJoints);
    std::vector<int> leftLegControlModes(numLeftLegJoints,VOCAB_CM_POSITION);
    if(! leftLegIControlMode2->setControlModes( leftLegControlModes.data() )){
        CD_ERROR("Problems setting position control mode of: left-Leg\n");
        return false;
    }
    rightLegIPositionControl2->getAxes(&numRightLegJoints);
    std::vector<int> rightLegControlModes(numRightLegJoints,VOCAB_CM_POSITION);
    if(! rightLegIControlMode2->setControlModes(rightLegControlModes.data())){
        CD_ERROR("Problems setting position control mode of: right-Leg\n");
        return false;    }

    /** **************************************************************************************
     * ******************************************************************************** **/

    yarp::os::Time::delay(1);

    //-- Conection between TestCap544 & ThreadImpl
    threadImpl.setNumJoints(numRightLegJoints,numLeftLegJoints);
    threadImpl.setIEncodersControl(rightLegIEncoders,leftLegIEncoders);
    threadImpl.setIPositionControl2(rightLegIPositionControl2,leftLegIPositionControl2);
    threadImpl.setIControlMode2(rightLegIControlMode2,leftLegIControlMode2);
    //threadImpl.setIVelocityControl2(rightLegIVelocityControl2,leftLegIVelocityControl2); // no se utiliza de momento
    threadImpl.setInputPorts(&portImu,&portft0,&portft1,&portft2,&portft3);

    threadImpl.start();

    return true;
}

/************************************************************************/
double TestCap544::getPeriod() {
    return 4.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool TestCap544::updateModule() {
    printf("TestCap51 alive...\n");
    return true;
}

/************************************************************************/
bool TestCap544::interruptModule() {
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
