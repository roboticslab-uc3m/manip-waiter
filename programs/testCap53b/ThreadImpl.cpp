// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ThreadImpl.hpp"

namespace roboticslab
{
/************************************************************************/
bool ThreadImpl::threadInit()
{
    printf("[success] Entrando en ratethread -> init/run\n");

    a = 0;
    b = 0;
    n = 1;

    zmp_ref = 0; // initial value and our zmp reference
    _ang_ref = 0; // initial value and our angle reference
    _ang_out = 0; // initial value and our output

    test = 0.01; // valor del ZMP al que se quiere testear

/*    // configuracion del orden de la ft 1/2
    id1 = OnlineSystemIdentification(1,2,1);
    num = new std::vector<double> (2);
    den = new std::vector<double> (3);*/

/*   // configuracion del orden de la ft 2/3
    id1 = OnlineSystemIdentification(2,3,1);
    num = new std::vector<double> (3);
    den = new std::vector<double> (4);*/

    // configuracion del orden de la ft 2/4
     id1 = OnlineSystemIdentification(2,4,0.99);
     num = new std::vector<double> (3);
     den = new std::vector<double> (5);

     return true;
}

/************************************************************************/
void ThreadImpl::run()
{
    while(!isStopping()) {

        // ------------------------------------------------------------------------
        if (a!=1)    {    // STEP 1 - Creating & Configuring CSV file
            confCSVfile();
            a=1;    }
        if (a==1 && b!=1)    {    // STEP 2 - waiting to start
            cout << "[push ENTER, please] ... STEP 2 " << endl;
            getchar();
            b=1;    }


        // ------------------------------------------------------------------------
        if (a==1 && b==1)   {   // STEP 3 - main code

            // Test escalon con rampa // generacion del ZMP_ref para test FT sensor
            // equaction - positive slope: zmp_ref = (test/60)*n - (test*10);
            // equaction - negative slope: zmp_ref = (-test/60)*n + ((test*15)+test);

            getInitialTime();
            readSensorsFT0(); // reading AnalogSensor FT0
            readSensorsFT1(); // reading AnalogSensor FT1
            zmpCompFT(); // ZMP_FT computation

            if (n <= 300)
            {
                zmp_ref = 0.0; // target
                zmp_ref = zmp_ref+0.0001*((rand() % 10 + 1)-5); // random function to include noise

                /** IDENTIFICATION   **/
                id1.UpdateSystem(zmp_ref*100,Xzmp_ft*100);
                id1.PrintZTransferFunction(0.02);
                id1.GetZTransferFunction(*num, *den);

                printData();
                saveInFileCsv();  // saving the information

            }

            else if (n >= 300 && n <= 360)  // test from 0.01 to 0.09 [m]
            {
                //zmp_ref = (test/60)*n - (test*10); // target
                //zmp_ref = (test/90)*n - (test*300/90); // target
                zmp_ref = (test/60)*n - (test*300/60); // target
                zmp_ref = zmp_ref+0.0001*((rand() % 10 + 1)-5); // random function to include noise

                evaluateLIPM(); // evaluacion the model and the angle output
                setJoints(); // applying the ankle movement

                /** IDENTIFICATION   **/
                id1.UpdateSystem(zmp_ref*100,Xzmp_ft*100);
                id1.PrintZTransferFunction(0.02);
                id1.GetZTransferFunction(*num, *den);

                printData();
                saveInFileCsv();  // saving the information
            }

            else if (n >= 360 && n <= 900)
            {
                zmp_ref = test; // target
                zmp_ref = zmp_ref+0.0001*((rand() % 10 + 1)-5); // random function to include noise

                /** IDENTIFICATION FUCTIONS  **/
                id1.UpdateSystem(zmp_ref*100,Xzmp_ft*100); // applying the new output/input
                id1.PrintZTransferFunction(0.02);
                id1.GetZTransferFunction(*num, *den);

                printData();
                saveInFileCsv();  // saving the information
                //n--;
            }

            else if (n >= 900 && n <= 960)
            {
                zmp_ref = (-test/60)*n + ((test*15)+test); // target
                zmp_ref = zmp_ref+0.0001*((rand() % 10 + 1)-5); // random function to include noise

                evaluateLIPM(); // generation of the angle output based on LIPM
                setJoints(); // applying the ankle movement

                printData();
            }

            else
                {zmp_ref = 0;}


            cout << endl << "Press Ctrl+C to exit..." << endl << endl;
            cout << "*******************************" << endl << endl;

            cout << n << endl << endl;
            n++;

            getCurrentTime();
            yarp::os::Time::delay(0.02);
        }
    }
}


/************************************************************************/
      //--  CONTROL  --//
/************************************************************************/
void ThreadImpl::evaluateLIPM()        /** Calculating OUTPUT (Qi) of the legs based on LIPM. **/
{
    _ang_out = -360*zmp_ref/ (2*PI*L); // Transforming from ZMP to Angle
}

/************************************************************************/
void ThreadImpl::setJoints()        /** Position control **/
{
    rightLegIPositionDirect->setPosition(4, _ang_out); // position in degrees
    leftLegIPositionDirect->setPosition(4, _ang_out);
}


/************************************************************************/
      //--  DATA ACQUISITION  --//
/************************************************************************/
void ThreadImpl::readSensorsFT0()       /** Reading input messages from FT0 SENSORS    **/
{
    //--- FT-Sensor 0 right leg

    yarp::sig::Vector ch0;
    int ret = ft0AnalogSensor->read(ch0); // lecture from sensor JR3 ch0 - right leg
    if (ret == yarp::dev::IAnalogSensor::AS_OK)
    {
        _RL._F.fx = ch0[0];
        _RL._F.fy = ch0[1];
        _RL._F.fz = ch0[2];
        _RL._T.mx = ch0[3];
        _RL._T.my = ch0[4];
        _RL._T.mz = ch0[5];

        //std::printf("Good read, got: %s\n",ch0.toString().c_str()); // print on terminal vector ch0
    }
}

/************************************************************************/
void ThreadImpl::readSensorsFT1()       /** Reading input messages from FT1 SENSORS    **/
{
    //--- FT-Sensor 1 left leg

    yarp::sig::Vector ch1;
    int ret = ft1AnalogSensor->read(ch1); // lecture from sensor JR3 ch1 - left leg
    if (ret == yarp::dev::IAnalogSensor::AS_OK)
    {
        _LL._F.fx = ch1[0];
        _LL._F.fy = ch1[1];
        _LL._F.fz = ch1[2];
        _LL._T.mx = ch1[3];
        _LL._T.my = ch1[4];
        _LL._T.mz = ch1[5];

        //std::printf("Good read, got: %s\n",ch1.toString().c_str()); // print on terminal vector ch1
    }
}

/************************************************************************/
void ThreadImpl::readSensorsIMU()       /** Reading input messages from IMU SENSORS    **/
{
        //--- Inertial-Sensor
    Bottle imu;
    portImu->read(imu); // lectura del sensor IMU
    ang_x = imu.get(0).asDouble(); // Angulo en X [deg]
    //ang_y = imu.get(1).asDouble(); // Angulo en Y [deg]
    //ang_z = imu.get(2).asDouble(); // Angulo en Z [deg]
    acc_x = imu.get(3).asDouble(); //Linear acceleration in X [m/s^2]
    x_sensor.push_front(acc_x);
    x_sensor.pop_back();
    //acc_y = imu.get(4).asDouble(); //Linear acceleration in Y [m/s^2]
    //y_sensor.push_front(acc_y);
    //y_sensor.pop_back();
    //acc_z = imu.get(5).asDouble(); //Linear acceleration in Z [m/s^2]
    //z_sensor.push_front(acc_z);
    //z_sensor.pop_back();
    //spd_x=imu.get(6).asDouble(); // Velocidad angular en X [deg/s]
    //spd_y=imu.get(7).asDouble(); // Velocidad angular en Y [deg/s]
    //spd_z=imu.get(8).asDouble(); // Velocidad angular en Z [deg/s]
    //mag_x=imu.get(9).asDouble(); // Campo magnetico en X
    //mag_y=imu.get(10).asDouble(); // Campo magnetico en Y
    //mag_z=imu.get(11).asDouble(); // Campo magnetico en Z

    //LOW-PASS FILTER
    ddx = 0.0;
    //ddy = 0.0;
    //ddz = 0.0;
    for(deque<double>::iterator it = x_sensor.begin(); it != x_sensor.end(); it++)
        ddx = ddx + *it;
    //for(deque<double>::iterator it = y_sensor.begin(); it != y_sensor.end(); it++)
        //ddy = ddy + *it;
    //for(deque<double>::iterator it = z_sensor.begin(); it != z_sensor.end(); it++)
        //ddz = ddz + *it;
    ddx = ddx / samples;
    //ddy = ddy / samples;
    //ddz = ddz / samples;

    //CONVERSION FROM IMU SENSOR COORDINATES TO ROBOT COORDINATES
     ddx_robot = ddx;
     //ddy_robot = -ddy;
     //ddz_robot = ddz;

}

/************************************************************************/
void ThreadImpl::zmpCompFT()        /** Calculating ZMP-FT of the body . **/
{
    //ZMP Equations : Double Support - FT

    _xzmp_ft0 = -(((_RL._T.my/10) + e*_RL._F.fx)) / _RL._F.fz; // xzmp0_ft in [m] right foot
    //_yzmp0_ft = -(((_RF._T.mx/10) + e*_RF._F.fy)) / _RF._F.fz; // xzmp0_ft in [m] right foot
    _xzmp_ft1 = -(((_LL._T.my/10) + e*_LL._F.fx)) / _LL._F.fz; // xzmp1_ft in [m] left foot
    //_yzmp1_ft = -(((_LF._T.mx/10) + e*_LF._F.fy)) / _LF._F.fz; // xzmp1_ft in [m] left foot

    _xzmp_ft01 = (_xzmp_ft0 * _RL._F.fz + _xzmp_ft1 * _LL._F.fz) / (_RL._F.fz + _LL._F.fz); // xzmp_ft in [m] robot
    //_yzmp01_ft = (_yzmp0_ft * _RF._F.fz + _yzmp1_ft * _LF._F.fz) / (_RF._F.fz + _LF._F.fz); // xzmp_ft in [m] robot

    // OFFSET FT - deleting the offset of _xzmp01_ft ( both ZMPs from sensor in frontal plane)
    if (n >=1 && n < 100){
        sum_x_ft = _xzmp_ft01 + sum_x_ft;
        offs_x_ft = sum_x_ft / n;
        printf("offset zmp X = %f\n", offs_x_ft);
    }

    Xzmp_ft  = _xzmp_ft01 - offs_x_ft; // frontal plane - final ZMP X [m]
    //Yzmp_ft = _yzmp01_ft - offs_y_ft; // saggital plane - final ZMP Y [m]

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
    Xzmp_imu = Xcom - (Zcom / ddz_robot) * ddx_robot; // frontal plane - final ZMP X [m]
    //Yzmp_imu = Ycom - (Zcom / ddz_robot) * ddy_robot;// saggital plane - final ZMP Y [m]
}



/************************************************************************/
      //--  LESS IMPORTANT  --//
/************************************************************************/
void ThreadImpl::printData()      /** Printing in Terminal    **/
{
    cout << endl << "El ZMP ref es: " << zmp_ref << endl;
    cout << endl << "El ZMP ft  es: " << Xzmp_ft << endl;
    cout << endl << "El ANG out es: " << _ang_out << endl;
    //cout << endl << "El angulo out es: " << _ang_out << endl;
}

/************************************************************************/
void ThreadImpl::saveInFileCsv()      /** Saving data in CSV file    **/
{
    fprintf(fp,"\n%.2f", act_time);

    fprintf(fp,",%.10f", _RL._F.fx); // f_x - sensor ft 0
    fprintf(fp,",%.10f", _RL._F.fz); // f_z - sensor ft 0
    fprintf(fp,",%.10f", _RL._T.my); // m_y - sensor ft 0

    fprintf(fp,",%.10f", _LL._F.fx); // f_x - sensor ft 1
    fprintf(fp,",%.10f", _LL._F.fz); // f_z - sensor ft 1
    fprintf(fp,",%.10f", _LL._T.my); // m_y - sensor ft 1

    fprintf(fp,",%.8f", zmp_ref); // ZMP reference (double support) (frontal plane)
    fprintf(fp,",%.8f", Xzmp_ft); // ZMP measured (double support) (frontal plane)
    fprintf(fp,",%.8f", _xzmp_ft0); // zmp (right foot)
    fprintf(fp,",%.8f", _xzmp_ft1); // zmp (left foot)

    fprintf(fp,",%.10f", num->data()[0]); // Z al 0 numerador
    fprintf(fp,",%.10f", num->data()[1]); // Z al 1 numerador
    fprintf(fp,",%.10f", num->data()[2]); // Z al 2 numerador

    fprintf(fp,",%.10f", den->data()[0]); // Z al 0 denominador
    fprintf(fp,",%.10f", den->data()[1]); // Z al 1 denominador
    fprintf(fp,",%.10f", den->data()[2]); // Z al 2 denominador
    fprintf(fp,",%.10f", den->data()[3]); // Z al 3 denominador
    fprintf(fp,",%.10f", den->data()[4]); // Z al 4 denominador

    fprintf(fp,",%i", n); // numero de interaciones
}

/************************************************************************/
void ThreadImpl::confCSVfile()      /** Configuring CSV file    **/
{
    cout << "[configuring] ... STEP 1 " << endl;
    fp = fopen("../data_identificationTeoTest.csv","w+");
    //fprintf(fp,"Time,Fx_ft0,Fz_ft0,My_ft0,Fx_ft1,Fz_ft1,My_ft1,Xzmp_ref,Xzmp_ft,zmp_RF,zmp_LF,num0,num1,den0,den1,den2,iter");  // para ft 1/2
    //fprintf(fp,"Time,Fx_ft0,Fz_ft0,My_ft0,Fx_ft1,Fz_ft1,My_ft1,Xzmp_ref,Xzmp_ft,zmp_RF,zmp_LF,num0,num1,num2,den0,den1,den2,den3,iter");  // para ft 2/3
    fprintf(fp,"Time,Fx_ft0,Fz_ft0,My_ft0,Fx_ft1,Fz_ft1,My_ft1,Xzmp_ref,Xzmp_ft,zmp_RF,zmp_LF,num0,num1,num2,den0,den1,den2,den3,den4,iter");  // para ft 2/4
    yarp::os::Time::delay(1);

    cout << "[success] data_identificationTeoTest.csv file configured." << endl;
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
void ThreadImpl::getCurrentTime()       /** Get Current Time    **/
{
    act_time = Time::now() - init_time;
    act_loop = Time::now() - init_loop;
}

/************************************************************************/
void ThreadImpl::evaluateModel()        /** Calculating OUTPUT (Qi) of the legs. **/
{
    // obtaining the angle error for the D-LIPM space state

/*    _evalLIPM.model(Xzmp_ft,zmp_ref);
    ka = 0.25 * zmp_ref + 9.95; // dudo entre zmp_ref o Xzmp_ft
    _ang_ref = (zmp_ref*(-J_G))/ (J_L*(ka-J_G));
    _ang_out =  _evalLIPM.ang_error_out + _ang_ref;*/

}

}   // namespace roboticslab
