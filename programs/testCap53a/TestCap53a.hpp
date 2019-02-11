// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __FM_TESTCAP53A_HPP__ 
#define __FM_TESTCAP53A_HPP__

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
 * @ingroup testCap53a
 *
 * @brief testCap53a.
 *
 */   
class TestCap53a : public RFModule {
private:
    ThreadImpl threadImpl;

    yarp::os::Port portft0;
    yarp::os::Port portft1;
    yarp::os::Port portft2;
    yarp::os::Port portft3;
    yarp::os::Port portImu;

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
    /** Trunk TorqueControl Interface */
    yarp::dev::ITorqueControl *trunkITorqueControl; // para control en torque

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
    /** Left Leg TorqueControl Interface */
    yarp::dev::ITorqueControl *leftLegITorqueControl; // para control en torque

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
    /** Right Leg TorqueControl Interface */
    yarp::dev::ITorqueControl *rightLegITorqueControl; // para control en torque

    // -------------------------------------------------------------------------------------

    /** Lelt Leg ControlLimits2 Interface */
    yarp::dev::IControlLimits *leftLegIControlLimits;
    /** Solver device **/
    yarp::dev::PolyDriver leftLegSolverDevice;
    roboticslab::ICartesianSolver *leftLegICartesianSolver;

    /** Right Leg ControlLimits2 Interface */
    yarp::dev::IControlLimits *rightLegIControlLimits;
    /** Solver device **/
    yarp::dev::PolyDriver rightLegSolverDevice;
    roboticslab::ICartesianSolver *rightLegICartesianSolver;


    bool interruptModule();
    double getPeriod();
    bool updateModule();

public:
    bool configure(ResourceFinder &rf);

    protected:



};

}  // namespace roboticslab

#endif // __FM_TESTCAP53A_HPP__
