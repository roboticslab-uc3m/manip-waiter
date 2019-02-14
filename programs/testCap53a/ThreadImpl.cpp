// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ThreadImpl.hpp"

namespace roboticslab
{
/************************************************************************/
bool ThreadImpl::threadInit()
{
    printf("[success] entrando en ratethread -> init/run\n");

    a = 0;
    b = 0;
    n = 1;

    zmp_ref = 0; // initial value and our zmp reference
    _ang_ref = 0; // initial value and our angle reference
    _ang_out = 0; // initial value and our output

    leftLegIEncoders->getAxes(&numLeftLegJoints);
    rightLegIEncoders->getAxes(&numRightLegJoints);

    Time::delay(2);
    return true;
}

/************************************************************************/
void ThreadImpl::run()
{
    while(!isStopping()) {

        if (a!=1)    {    // STEP 1 - Creating & Configuring CSV file
            confCSVfile();


            a=1;    }
        if (a==1 && b!=1)    {    // STEP 2 - Opening & Connecting Ports
            openingPorts();
            getchar();
            b=1;    }

        // ------------------------------------------------------------------------
        if (a==1 && b==1)
        {
            // STEP 3 - main code

            // Test escalon con rampa // generacion del ZMP_ref para test FT sensor
            // equaction: zmp_ref = (0.0X / 30) * n - 0.X
            // where x is the ref value
            // example_1: zmp_ref = (0.05/30)*n - 0.5 ------> for zmp_ref = 0.05 meters
            // example_2: zmp_ref = (0.09/30)*n - 0.9 ------> for zmp_ref = 0.09 meters

            if (n <= 300){zmp_ref = 0.0;}
            else if (n >= 300 && n <= 330){zmp_ref = (0.09/30)*n - 0.9;} // test from 0.01 to 0.09 [m]
            else {zmp_ref = zmp_ref;}

            getInitialTime();

            //readSensorsFT0();
            //readSensorsFT1();

            std::vector<double> rightLegQs(numRightLegJoints);
            std::vector<double> leftLegQs(numLeftLegJoints);
            if (!rightLegIEncoders->getEncoders(rightLegQs.data()))            {
                CD_ERROR("getEncoders failed, unable to check joint limits.\n");
                return;            }
            if (!leftLegIEncoders->getEncoders(leftLegQs.data()))            {
                CD_ERROR("getEncoders failed, unable to check joint limits.\n");
                return;            }

            printf("[success] hasta aqui hemos llegado\n");

            handleGcmp(rightLegQs,leftLegQs);

/*            if (n>300)  {
                zmpCompFT(); // calculation of the ZMP_FT
                evaluateModel(); // evaluacion the model and the angle output
                setJoints(); // applying the ankle movement
            }*/

            printData();
            cout << endl << "Press ENTER to exit..." << endl;
            cout << "*******************************" << endl << endl;
            saveInFileCsv();  // saving the information ¡
            n++;
            cout << n << endl << endl;

            getCurrentTime();
        }
    }
}

/************************************************************************/
void ThreadImpl::confCSVfile()      /** Configuring CSV file    **/
{
    cout << "[configuring] ... STEP 1 " << endl;
    fp = fopen("../data_testingDLIPM.csv","w+");
    fprintf(fp,"Time,Fx_ft0,Fz_ft0,My_ft0,Fx_ft1,Fz_ft1,My_ft1,Xzmp_ft,zmp_RF,zmp_LF,iter");
    yarp::os::Time::delay(1);

    cout << "[success] data_dlipm_threat.csv file configured." << endl;
}

/************************************************************************/
void ThreadImpl::openingPorts()     /** Opening Ports & Connecting with sensor programs **/
{
    cout << "[configuring] ... STEP 2 " << endl;

    //-- OPEN YARP PORTS
    //portImu->open("/bodyBal/inertial:i");
    portFt0->open("/bodyBal/jr3ch0:i");
    portFt1->open("/bodyBal/jr3ch1:i");
    //portFt2->open("/bodyBal/jr3ch2:i");
    //portFt3->open("/bodyBal/jr3ch3:i");


    //-- CONNECTIONS PORTS

    // ft right foot
    Network::connect("/jr3/ch0:o","/bodyBal/jr3ch0:i");
    if ( NetworkBase::isConnected("/jr3/ch0:o","/bodyBal/jr3ch0:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /bodyBal/jr3ch0:i." << endl;
    } else cout << "[success] Connected to /jr3ch0:i." << endl;
    yarp::os::Time::delay(0.5);
    // ft left foot
    Network::connect("/jr3/ch1:o","/bodyBal/jr3ch1:i");
    if ( NetworkBase::isConnected("/jr3/ch1:o","/bodyBal/jr3ch1:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /bodyBal/jr3ch1:i." << endl;
    } else cout << "[success] Connected to /jr3ch1:i." << endl;
    yarp::os::Time::delay(0.5);
/*    // ft right hand
    Network::connect("/jr3/ch2:o","/bodyBal/jr3ch2:i");
    if ( NetworkBase::isConnected("/jr3/ch2:o","/bodyBal/jr3ch2:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /bodyBal/jr3ch2:i." << endl;
    } else cout << "[success] Connected to /jr3ch2:i." << endl;
    yarp::os::Time::delay(0.5);
    // ft left hand
    Network::connect("/jr3/ch3:o","/bodyBal/jr3ch3:i");
    if ( NetworkBase::isConnected("/jr3/ch3:o","/bodyBal/jr3ch3:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /bodyBal/jr3ch3:i." << endl;
    } else cout << "[success] Connected to /jr3ch3:i." << endl;
    yarp::os::Time::delay(0.5);
    // imu trunk
    Network::connect("/inertial", "/bodyBal/inertial:i");
    if ( NetworkBase::isConnected("/inertial", "/bodyBal/inertial:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /bodyBal/inertial:i." << endl;
    } else cout << "[success] Connected to IMU." << endl;
    yarp::os::Time::delay(0.5);*/

    return;
}

/************************************************************************/
void ThreadImpl::getInitialTime()       /** Get Initial Time    **/
{
    if (n==1){init_time = Time::now();}
    init_loop = Time::now();
    it_time = init_loop - it_prev;
    it_prev = init_loop;
}

/************************************************************************/
void ThreadImpl::readSensorsFT0()       /** Reading input messages from FT0 SENSORS    **/
{
        //--- FT-Sensor 0 right leg
    Bottle ch0;
    portFt0->read(ch0); // lectura del sensor JR3 ch0 - right foot
    _RF._F.fx = ch0.get(0).asDouble();
    _RF._F.fy = ch0.get(1).asDouble();
    _RF._F.fz = ch0.get(2).asDouble();
    _RF._T.mx = ch0.get(3).asDouble();
    _RF._T.my = ch0.get(4).asDouble();
    _RF._T.mz = ch0.get(5).asDouble();

}

/************************************************************************/
void ThreadImpl::readSensorsFT1()       /** Reading input messages from FT1 SENSORS    **/
{
        //--- FT-Sensor 1 left leg
    Bottle ch1;
    portFt1->read(ch1); // lectura del sensor JR3 ch1 - left foot
    _LF._F.fx = ch1.get(0).asDouble();
    _LF._F.fy = ch1.get(1).asDouble();
    _LF._F.fz = ch1.get(2).asDouble();
    _LF._T.mx = ch1.get(3).asDouble();
    _LF._T.my = ch1.get(4).asDouble();
    _LF._T.mz = ch1.get(5).asDouble();

}

/************************************************************************/
void ThreadImpl::readSensorsFT2()       /** Reading input messages from FT2 SENSORS    **/
{
        //--- FT-Sensor 2 right hand
    Bottle ch2;
    portFt2->read(ch2); // lectura del sensor JR3 ch2 - right hand
    _RH._F.fx = ch2.get(0).asDouble();
    _RH._F.fy = ch2.get(1).asDouble();
    _RH._F.fz = ch2.get(2).asDouble();
    _RH._T.mx = ch2.get(3).asDouble();
    _RH._T.my = ch2.get(4).asDouble();
    _RH._T.mz = ch2.get(5).asDouble();

}

/************************************************************************/
void ThreadImpl::readSensorsFT3()       /** Reading input messages from FT3 SENSORS    **/
{
    //--- FT-Sensor 3 left hand
    Bottle ch3;
    portFt3->read(ch3); // lectura del sensor JR3 ch3 - left hand
    _LH._F.fx = ch3.get(0).asDouble();
    _LH._F.fy = ch3.get(1).asDouble();
    _LH._F.fz = ch3.get(2).asDouble();
    _LH._T.mx = ch3.get(3).asDouble();
    _LH._T.my = ch3.get(4).asDouble();
    _LH._T.mz = ch3.get(5).asDouble();

}

/************************************************************************/
void ThreadImpl::readSensorsIMU()       /** Reading input messages from IMU SENSORS    **/
{
        //--- Inertial-Sensor
    Bottle imu;
    portImu->read(imu); // lectura del sensor IMU
    ang_x = imu.get(0).asDouble(); // Angulo en X [deg]
    ang_y = imu.get(1).asDouble(); // Angulo en Y [deg]
    ang_z = imu.get(2).asDouble(); // Angulo en Z [deg]
    acc_x = imu.get(3).asDouble(); //Linear acceleration in X [m/s^2]
    x_sensor.push_front(acc_x);
    x_sensor.pop_back();
    acc_y = imu.get(4).asDouble(); //Linear acceleration in Y [m/s^2]
    y_sensor.push_front(acc_y);
    y_sensor.pop_back();
    acc_z = imu.get(5).asDouble(); //Linear acceleration in Z [m/s^2]
    z_sensor.push_front(acc_z);
    z_sensor.pop_back();
    spd_x=imu.get(6).asDouble(); // Velocidad angular en X [deg/s]
    spd_y=imu.get(7).asDouble(); // Velocidad angular en Y [deg/s]
    spd_z=imu.get(8).asDouble(); // Velocidad angular en Z [deg/s]
    //mag_x=imu.get(9).asDouble(); // Campo magnetico en X
    //mag_y=imu.get(10).asDouble(); // Campo magnetico en Y
    //mag_z=imu.get(11).asDouble(); // Campo magnetico en Z

    //LOW-PASS FILTER
    ddx = 0.0;
    ddy = 0.0;
    ddz = 0.0;
    for(deque<double>::iterator it = x_sensor.begin(); it != x_sensor.end(); it++)
        ddx = ddx + *it;
    for(deque<double>::iterator it = y_sensor.begin(); it != y_sensor.end(); it++)
        ddy = ddy + *it;
    for(deque<double>::iterator it = z_sensor.begin(); it != z_sensor.end(); it++)
        ddz = ddz + *it;
    ddx = ddx / samples;
    ddy = ddy / samples;
    ddz = ddz / samples;

    //CONVERSION FROM IMU SENSOR COORDINATES TO ROBOT COORDINATES
     ddx_robot = ddx;
     ddy_robot = -ddy;
     ddz_robot = ddz;

}

/************************************************************************/
void ThreadImpl::getCurrentTime()       /** Get Current Time    **/
{
    act_time = Time::now() - init_time;
    act_loop = Time::now() - init_loop;
}

/************************************************************************/
void ThreadImpl::zmpCompFT()        /** Calculating ZMP-FT of the body . **/
{

    //ZMP Equations : Double Support - FT

/*      //Con el coeficiente 1000 - unidades en milimetros
    _xzmp0 = -(((_my0/10) + e*_fx0)*1000) / _fz0; // xzmp0 in [mm]
    _yzmp0 = (((_mx0/10) + e*_fy0)*1000) / _fz0; // yzmp0 in [mm]

    _xzmp1 = -(((_my1/10) + e*_fx1)*1000) / _fz1; // xzmp1 in [mm]
    _yzmp1 = (((_mx1/10) + e*_fy1)*1000) / _fz1; // yzmp1 in [mm]
*/

    //Sin el coeficiente 1000 - unidades en metros
    _xzmp_ft0 = -(((_RF._T.my/10) + e*_RF._F.fx)) / _RF._F.fz; // xzmp0_ft in [m] right foot
    //_yzmp0_ft = -(((_RF._T.mx/10) + e*_RF._F.fy)) / _RF._F.fz; // xzmp0_ft in [m] right foot
    _xzmp_ft1 = -(((_LF._T.my/10) + e*_LF._F.fx)) / _LF._F.fz; // xzmp1_ft in [m] left foot
    //_yzmp1_ft = -(((_LF._T.mx/10) + e*_LF._F.fy)) / _LF._F.fz; // xzmp1_ft in [m] left foot

    _xzmp_ft01 = (_xzmp_ft0 * _RF._F.fz + _xzmp_ft1 * _LF._F.fz) / (_RF._F.fz + _LF._F.fz); // xzmp_ft in [m] robot
    //_yzmp01_ft = (_yzmp0_ft * _RF._F.fz + _yzmp1_ft * _LF._F.fz) / (_RF._F.fz + _LF._F.fz); // xzmp_ft in [m] robot

    // OFFSET FT - eliminando el offset de _xzmp01_ft (ZMP de ambos sensores en X)
    if (n >=1 && n < 50){
        sum_x_ft = _xzmp_ft01 + sum_x_ft;
        offs_x_ft = sum_x_ft / n;
        printf("offs = %f\n", offs_x_ft);
    }

    Xzmp_ft  = _xzmp_ft01 - offs_x_ft; // frontal plane
    //Yzmp_ft = _yzmp01_ft - offs_y_ft; // saggital plane

    if ((_xzmp_ft01 != _xzmp_ft01) || (_yzmp_ft01 != _yzmp_ft01)){
        printf ("Warning: No zmp data\n");
    }

}

/************************************************************************/
void ThreadImpl::zmpCompIMU()       /** Calculating ZMP-IMU of the body . **/
{

    //ZMP Equations : Double Support - IMU

    // OFFSET IMU - eliminando el offset de ddx_robot (aceleracion en X)
    if (n >=100 && n < 250){
        sum_x_imu = ddx_robot + sum_x_imu;
        offs_x_imu = sum_x_imu / (n-150);
        printf("offs = %f\n", offs_x_imu);
    }

    ddx_robot = ddx_robot - offs_x_imu; // frontal plane
    //ddy_robot = ddy_robot - offs_y_imu; // saggital plane
    //ZERO MOMENT POINT COMPUTATION - IMU
    Xzmp_imu = Xcom - (Zcom / ddz_robot) * ddx_robot; //ZMP X coordinate [m]
    //Yzmp_imu = Ycom - (Zcom / ddz_robot) * ddy_robot; //ZMP Y coordinate [m]

}

/************************************************************************/
void ThreadImpl::evaluateModel()        /** Calculating OUTPUT (Qi) of the legs. **/
{
/**    // obtaining the angle error for the D-LIPM space state
    _evalLIPM.model(Xzmp_ft,zmp_ref);

    ka = 0.25 * zmp_ref + 9.95; // dudo entre zmp_ref o Xzmp_ft
    _ang_ref = (zmp_ref*(-G))/ (L*(ka-G));

/*    como otra posibilidad para calcular:  _angle_ref
    if ( ! leftArmIEncoders->getEncoders( encLegs.data() ) )    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;    }
    _angle_ref = encLegs[4];
*

    _ang_out =  _evalLIPM.ang_error_out + _ang_ref;
**/

    /** EVALUACION MODELO LOLI **/
    _evalLIPM.model(Xzmp_ft,zmp_ref);

    _ang_out = -(_evalLIPM.y-(4.3948*pow(_evalLIPM.y,2))+0.23*_evalLIPM.y)/0.0135;

    vel = _evalLIPM.dy * (1/L) * (180/PI); //velocity in degrees per second
    //vel = 0.35* _evalLIPM.dy * (1/L) * (180/PI); //velocity in degrees per second

}

/************************************************************************/
void ThreadImpl::setJoints()        /** Position control **/
{
    //rightLegIPositionControl2->positionMove(4, _ang_out); // position in degrees
    //leftLegIPositionControl2->positionMove(4, _ang_out);
  if (n<450){
    rightLegIVelocityControl2->velocityMove(4, -vel); // velocity in degrees per second
    leftLegIVelocityControl2->velocityMove(4, -vel);
  }
}

/************************************************************************/
void ThreadImpl::handleGcmp(const std::vector<double> &rightLegQs, const std::vector<double> &leftLegQs)
{
    std::vector<double> rightLegTorques(numRightLegJoints);
    std::vector<double> leftLegTorques(numLeftLegJoints);

    if (!rightLegICartesianSolver->invDyn(rightLegQs, rightLegTorques))    {
        CD_WARNING("invDyn failed, not updating control this iteration.\n");
        return;    }
    if (!rightLegITorqueControl->setRefTorques(rightLegTorques.data()))    {
        CD_WARNING("setRefTorques failed, not updating control this iteration.\n");    }

    if (!leftLegICartesianSolver->invDyn(leftLegQs, leftLegTorques))    {
        CD_WARNING("invDyn failed, not updating control this iteration.\n");
        return;    }
    if (!leftLegITorqueControl->setRefTorques(leftLegTorques.data()))    {
        CD_WARNING("setRefTorques failed, not updating control this iteration.\n");    }

}

/************************************************************************/
void ThreadImpl::printData()
{
/*        cout << endl << "El angulo 1 es: " << _angle_ref_a << endl;
    cout << endl << "El angulo 2 es: " << _angle_ref_b << endl;
    cout << endl << "El angulo 3 es: " << _angle_ref_c << endl;
    cout << endl << "El angulo 4 es: " << _angle_ref_d << endl;
    cout << endl << "El angulo 4 es: " << g << endl;
    cout << endl << "El angulo 4 es: " << ka << endl;
*/
    //cout << endl << "El ANKLE pid es: " << pid_output_ankle << endl;
    //cout << endl << "El HIP pid es: " << pid_output_hip << endl;
    //cout << endl << "El ZMP REF es: " << setpoint << endl;

    cout << endl << "El ZMP REF es: " << zmp_ref << endl;
    cout << endl << "El ZMP FT es: " << Xzmp_ft << endl;
    cout << endl << "El ANG OUT es: " << _ang_out << endl;
    cout << endl << "La Vel OUT es: " << vel << endl;

    //cout << endl << "La capture_point es: " << capture_point << endl;
    //cout << endl << "ZMP_Error_Loli = ("<< _eval_x._zmp_error << ") [mm]" << endl;
    //cout << endl << "ZMP model es: " << _eval_x.y << endl;
    //cout << endl << "Num es: " << _num << endl;  _u_ref
    //cout << endl << "El _u_ref x es: " << _eval_x._u_ref << endl;
    //cout << endl << "El _angle_error x es: " << _eval_x._angle_error << endl;

}

/************************************************************************/
void ThreadImpl::saveInFileCsv()
{

    fprintf(fp,"\n%.2f", act_time);

    fprintf(fp,",%.10f", _RF._F.fx); // f_x - sensor ft 0
    fprintf(fp,",%.10f", _RF._F.fz); // f_z - sensor ft 0
    fprintf(fp,",%.10f", _RF._T.my); // m_y - sensor ft 0

    fprintf(fp,",%.10f", _LF._F.fx); // f_x - sensor ft 1
    fprintf(fp,",%.10f", _LF._F.fz); // f_z - sensor ft 1
    fprintf(fp,",%.10f", _LF._T.my); // m_y - sensor ft 1

    fprintf(fp,",%.8f", Xzmp_ft); // ZMP body (double support) (frontal plane)
    fprintf(fp,",%.8f", _xzmp_ft0); // zmp (right foot)
    fprintf(fp,",%.8f", _xzmp_ft1); // zmp (leftt foot)

    fprintf(fp,",%i", n);

}

/************************************************************************/
void ThreadImpl::setIEncodersControl(IEncoders *iRightLegEncoders,IEncoders *iLeftLegEncoders,IEncoders *iTrunkEncoders)
{
    this->rightLegIEncoders = iRightLegEncoders;
    this->leftLegIEncoders = iLeftLegEncoders;
    this->trunkIEncoders = iTrunkEncoders;
}

/************************************************************************/
void ThreadImpl::setIPositionControl2(IPositionControl2 *iRightLegPositionControl2,IPositionControl2 *iLeftLegPositionControl2,IPositionControl2 *iTrunkPositionControl2)
{
    this->rightLegIPositionControl2 = iRightLegPositionControl2;
    this->leftLegIPositionControl2 = iLeftLegPositionControl2;
    this->trunkIPositionControl2 = iTrunkPositionControl2;
}

/************************************************************************/
void ThreadImpl::setIVelocityControl2(IVelocityControl2 *iRightLegVelocityControl2,IVelocityControl2 *iLeftLegVelocityControl2,IVelocityControl2 *iTrunkVelocityControl2)
{
    this->rightLegIVelocityControl2 = iRightLegVelocityControl2;
    this->leftLegIVelocityControl2 = iLeftLegVelocityControl2;
    this->trunkIVelocityControl2 = iTrunkVelocityControl2;
}

/************************************************************************/
void ThreadImpl::setITorqueControl(ITorqueControl *iRightLegITorqueControl,ITorqueControl *iLeftLegITorqueControl,ITorqueControl *iTrunkITorqueControl) // no se utiliza de momento
{
    this->rightLegITorqueControl = iRightLegITorqueControl;
    this->leftLegITorqueControl = iLeftLegITorqueControl;
    this->trunkITorqueControl = iTrunkITorqueControl;
}

/************************************************************************/
void ThreadImpl::setInputPorts(Port *inputPortImu,Port *inputPortFt0,Port *inputPortFt1,Port *inputPortFt2,Port *inputPortFt3)
{
    this->portImu = inputPortImu;
    this->portFt0 = inputPortFt0;
    this->portFt1 = inputPortFt1;
    this->portFt2 = inputPortFt2;
    this->portFt3 = inputPortFt3;
}

/************************************************************************/
void ThreadImpl::setICartesianSolver(ICartesianSolver *iRightLegCartesianSolver,ICartesianSolver *iLeftLegCartesianSolver,ICartesianSolver *iTrunkCartesianSolver)
{
    this->rightLegICartesianSolver = iRightLegCartesianSolver;
    this->leftLegICartesianSolver = iLeftLegCartesianSolver;
    this->trunkICartesianSolver = iTrunkCartesianSolver;
}

}   // namespace roboticslab
