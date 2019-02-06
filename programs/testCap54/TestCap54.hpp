// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FM_TESTCAP54_HPP__ 
#define __FM_TESTCAP54_HPP__

#include <math.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <fstream>
#include <deque>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <cmath>
#include "ColorDebug.hpp"
#include <iomanip>

#include "KinematicRepresentation.hpp"
#include "ICartesianSolver.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace roboticslab;

#include "ThreadImpl.hpp"

#define DEFAULT_ROBOT "/teo"    // Por defecto, usaremos el robot // -> con simulador: "/teoSim"

namespace roboticslab
{

/**
 * @ingroup testCap54
 *
 * @brief testCap54.
 *
 */   
class TestCap54 : public RFModule {
private:
    ThreadImpl threadImpl;

    yarp::os::Port portImu;

    /** Axes number **/
    int numHeadJoints;
    /** Head Device */
    yarp::dev::PolyDriver headDevice;
    /** Encoders **/
    yarp::dev::IEncoders *headIEncoders;
    /** Head ControlMode2 Interface */
    yarp::dev::IControlMode2 *headIControlMode2;
    /** Head PositionControl2 Interface */
    yarp::dev::IPositionControl2 *headIPositionControl2; // para control en posicion
    /** Head VelocityControl2 Interface */
    yarp::dev::IVelocityControl2 *headIVelocityControl2; // para control en velocidad

    /** Axes number **/
    int numLeftArmJoints;
    /** Left Arm Device */
    yarp::dev::PolyDriver leftArmDevice;
    /** Encoders **/
    yarp::dev::IEncoders *leftArmIEncoders;
    /** Left Arm ControlMode2 Interface */
    yarp::dev::IControlMode2 *leftArmIControlMode2;
    /** Left Arm PositionControl2 Interface */
    yarp::dev::IPositionControl2 *leftArmIPositionControl2; // para control en posicion
    /** Left Arm VelocityControl2 Interface */
    yarp::dev::IVelocityControl2 *leftArmIVelocityControl2; // para control en velocidad
    /** FT 3 Sensor Device */
    yarp::dev::PolyDriver ft3SensorDevice;
    /** FT 3 AnalogSensor Interface */
    yarp::dev::IAnalogSensor *iFT3AnalogSensor;

    /** Axes number **/
    int numRightArmJoints;
    /** Right Arm Device */
    yarp::dev::PolyDriver rightArmDevice;
    /** Encoders **/
    yarp::dev::IEncoders *rightArmIEncoders;
    /** Right Arm ControlMode2 Interface */
    yarp::dev::IControlMode2 *rightArmIControlMode2;
    /** Right Arm PositionControl2 Interface */
    yarp::dev::IPositionControl2 *rightArmIPositionControl2; // para control en posicion
    /** Right Arm VelocityControl2 Interface */
    yarp::dev::IVelocityControl2 *rightArmIVelocityControl2; // para control en velocidad
    /** FT 2 Sensor Device */
    yarp::dev::PolyDriver ft2SensorDevice;
    /** FT 2 AnalogSensor Interface */
    yarp::dev::IAnalogSensor *iFT2AnalogSensor;

    // ----------------------------------------------------------------------------------

    /** Axes number **/
    int numTrunkJoints;
    /** Trunk Device */
    yarp::dev::PolyDriver trunkDevice;
    /** Encoders **/
    yarp::dev::IEncoders *trunkIEncoders;
    /** Trunk ControlMode2 Interface */
    yarp::dev::IControlMode2 *trunkIControlMode2;
    /** Trunk PositionControl2 Interface */
    yarp::dev::IPositionControl2 *trunkIPositionControl2; // para control en posicion
    /** Trunk VelocityControl2 Interface */
    yarp::dev::IVelocityControl2 *trunkIVelocityControl2; // para control en velocidad

    /** Axes number **/
    int numLeftLegJoints;
    /** Left Leg Device */
    yarp::dev::PolyDriver leftLegDevice;
    /** Encoders **/
    yarp::dev::IEncoders *leftLegIEncoders;
    /** Left Leg ControlMode2 Interface */
    yarp::dev::IControlMode2 *leftLegIControlMode2;
    /** Left Leg PositionControl2 Interface */
    yarp::dev::IPositionControl2 *leftLegIPositionControl2; // para control en posicion
    /** Left Leg VelocityControl2 Interface */
    yarp::dev::IVelocityControl2 *leftLegIVelocityControl2; // para control en velocidad
    /** FT 1 Sensor Device */
    yarp::dev::PolyDriver ft1SensorDevice;
    /** FT 1 AnalogSensor Interface */
    yarp::dev::IAnalogSensor *iFT1AnalogSensor;

    /** Axes number **/
    int numRightLegJoints;
    /** Right Leg Device */
    yarp::dev::PolyDriver rightLegDevice;
    /** Encoders **/
    yarp::dev::IEncoders *rightLegIEncoders;
    /** Right Leg ControlMode2 Interface */
    yarp::dev::IControlMode2 *rightLegIControlMode2;
    /** Right Leg PositionControl2 Interface */
    yarp::dev::IPositionControl2 *rightLegIPositionControl2; // para control en posicion
    /** Right Leg VelocityControl2 Interface */
    yarp::dev::IVelocityControl2 *rightLegIVelocityControl2; // para control en velocidad
    /** FT 0 Sensor Device */
    yarp::dev::PolyDriver ft0SensorDevice;
    /** FT 0 AnalogSensor Interface */
    yarp::dev::IAnalogSensor *iFT0AnalogSensor;

    // -------------------------------------------------------------------------------------

    /** Lelt Arm ControlLimits2 Interface */
    yarp::dev::IControlLimits *leftArmIControlLimits;
    /** Solver device **/
    yarp::dev::PolyDriver leftArmSolverDevice;
    roboticslab::ICartesianSolver *leftArmICartesianSolver;
    /** Forward Kinematic function **/
    bool getleftArmFwdKin(std::vector<double> *currentX);

    /** Right Arm ControlLimits2 Interface */
    yarp::dev::IControlLimits *rightArmIControlLimits;
    /** Solver device **/
    yarp::dev::PolyDriver rightArmSolverDevice;
    roboticslab::ICartesianSolver *rightArmICartesianSolver;
    /** Forward Kinematic function **/
    bool getRightArmFwdKin(std::vector<double> *currentX);

    bool interruptModule();
    double getPeriod();
    bool updateModule();

public:
    bool configure(ResourceFinder &rf);

    protected:



};

}  // namespace roboticslab

#endif // __FM_TESTCAP54_HPP__
