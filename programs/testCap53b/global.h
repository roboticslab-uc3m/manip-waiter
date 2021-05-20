#ifndef __GLOBAL_H__
#define __GLOBAL_H__

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

//General constants
#define PI  3.14159265358979323846
#define G   9.81 // Gravity [m/s²]
#define TS  0.03 // Time Samples [s]
#define L   0.599 // Pendulum Longitude [m] loli 0.8927
#define M   62.589 // Robot mass [kg]
#define e   0.03225 // Height FT sensor // Loli 0.0194

//Low-pass Filter
#define samples 30 //Number of samples for computing average

//Origin of coordinates established in the middle point between the feet
#define Xcom 0 //Distance to COM in X axis [cm]
#define Ycom 0 //Distance to COM in Y axis [cm]
#define Zcom 0.683 //Distance to COM in Z axis [cm] - Zcom 103.6602 JUANLO

//PID parameters
#define dt 0.05 //Loop interval time [s]
#define maximo 10 //Maximum output value
#define minimo -10 //Minimum output value
//Ankle parameters
#define Kp_ankle 50 //Proportional gain
#define Kd_ankle 5 //Derivative gain
#define Ki_ankle 1 //Integral gain
//Hip parameters
#define Kp_hip 50 //Proportional gain
#define Kd_hip 5 //Derivative gain
#define Ki_hip 1 //Integral gain

#endif // __GLOBAL_H__
