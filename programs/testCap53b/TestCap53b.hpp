// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FM_TESTCAP53b_HPP__ 
#define __FM_TESTCAP53b_HPP__

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

#include <yarp/sig/Vector.h>

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
 * @ingroup testCap53b
 *
 * @brief testCap53b.
 *
 */   
class TestCap53b : public RFModule {
private:
    ThreadImpl threadImpl;

    yarp::os::Port portImu;

    /** Axes number **/
    int numHeadJoints;
    /** Head Device */
    yarp::dev::PolyDriver headDevice;
    /** Encoders **/
    yarp::dev::IEncoders *headIEncoders;
    /** Head ControlMode Interface */
    yarp::dev::IControlMode *headIControlMode;
    /** Head PositionControl Interface */
    yarp::dev::IPositionControl *headIPositionControl; // para control en posicion
    /** Head VelocityControl Interface */
    yarp::dev::IVelocityControl *headIVelocityControl; // para control en velocidad

    /** Axes number **/
    int numLeftArmJoints;
    /** Left Arm Device */
    yarp::dev::PolyDriver leftArmDevice;
    /** Encoders **/
    yarp::dev::IEncoders *leftArmIEncoders;
    /** Left Arm ControlMode Interface */
    yarp::dev::IControlMode *leftArmIControlMode;
    /** Left Arm PositionControl Interface */
    yarp::dev::IPositionControl *leftArmIPositionControl; // para control en posicion
    /** Left Arm VelocityControl Interface */
    yarp::dev::IVelocityControl *leftArmIVelocityControl; // para control en velocidad
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
    /** Right Arm ControlMode Interface */
    yarp::dev::IControlMode *rightArmIControlMode;
    /** Right Arm PositionControl Interface */
    yarp::dev::IPositionControl *rightArmIPositionControl; // para control en posicion
    /** Right Arm VelocityControl Interface */
    yarp::dev::IVelocityControl *rightArmIVelocityControl; // para control en velocidad
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
    /** Trunk ControlMode Interface */
    yarp::dev::IControlMode *trunkIControlMode;
    /** Trunk PositionControl Interface */
    yarp::dev::IPositionControl *trunkIPositionControl; // para control en posicion
    /** Trunk VelocityControl Interface */
    yarp::dev::IVelocityControl *trunkIVelocityControl; // para control en velocidad

    /** Axes number **/
    int numLeftLegJoints;
    /** Left Leg Device */
    yarp::dev::PolyDriver leftLegDevice;
    /** Encoders **/
    yarp::dev::IEncoders *leftLegIEncoders;
    /** Left Leg ControlMode Interface */
    yarp::dev::IControlMode *leftLegIControlMode;
    /** Left Leg PositionControl Interface */
    yarp::dev::IPositionControl *leftLegIPositionControl; // para control en posicion
    /** Left Leg VelocityControl Interface */
    yarp::dev::IVelocityControl *leftLegIVelocityControl; // para control en velocidad
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
    /** Right Leg ControlMode Interface */
    yarp::dev::IControlMode *rightLegIControlMode;
    /** Right Leg PositionControl Interface */
    yarp::dev::IPositionControl *rightLegIPositionControl; // para control en posicion
    /** Right Leg VelocityControl Interface */
    yarp::dev::IVelocityControl *rightLegIVelocityControl; // para control en velocidad
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

#endif // __FM_TESTCAP53b_HPP__
