// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ThreadImpl.hpp"

namespace roboticslab
{
/************************************************************************/
bool ThreadImpl::threadInit() {
    printf("[success] entrando en ratethread -> init/run\n");

    a = 0;
    b = 0;
    c = 0;
    e = 0;
    f = 0;
    w = 0;
    n = 1;

    _d_X = 0.027; // distance in metres
    _l = 0.05;  // distance in metres

    quat.resize(4);
    quatC.resize(4);
    preFF.resize(4);
    FF.resize(4);

    _rzmp = 0; // initial
    _rzmp_b = 0;
    x_zmp_b = 0;
    y_zmp_b = 0;

    _off._F.fx = 0;
    _off._F.fy = 0;
    _off._F.fz = 0;
    _off._M.mx = 0;
    _off._M.my = 0;
    _off._M.mz = 0;

    homeX.resize(7);
    iniQ.resize(7);
    iniX.resize(7);
    curQ.resize(7);
    curX.resize(7);
    curX_AAS.resize(6);
    desX.resize(7);
    desX_AAS.resize(6);
    desQ.resize(7);
    befQ.resize(7);

    std::vector<double> armSpeeds(7,10.0); // 7,30.0
    if(!leftArmIPositionControl->setRefSpeeds(armSpeeds.data())){
        printf("[Error] Problems setting reference speed on left-arm joints.\n");
        return false;
    }
    std::vector<double> armAccelerations(7,10.0); // 7,30.0
    if(!leftArmIPositionControl->setRefAccelerations(armAccelerations.data())){
        printf("[Error] Problems setting reference acceleration on left-arm joints.\n");
        return false;
    }

return true;
}

/************************************************************************/
void ThreadImpl::run()
{
    while(!isStopping()) {

        if (a!=1)    {
            confCSVfile();  // STEP 1 - Creating & Configuring CSV file
            a=1;    }
        if (a==1 && b!=1)    {
            saveHomePrajectory();   // STEP 2 - Moving the arm/trunk to the stability pose
            b=1;    }
        if (a==1 && b==1 && e!=250)    {
            checkBottleReady();    } // STEP 3 - Waiting to put the bottle over the tray

        // ------------------------------------------------------------------------
        if (a==1 && b==1 && e==250)   {     // STEP 4 - main code

            getInitialTime();

            readSensorsFT3();
            readSensorsIMU();

            axesTransform1(); // SDC_jr3 -> SDC_root (permanente)
            axesTransform2(); // SDC_root -> SDC(theta) (variable)

            zmpComp(); // calculo del ZMP_FT
            LIPM3d(); // Calculo y generacion de la actuacion en funcion del ZMP_tray

            printData(); // muestreo de datos por terminal
            saveInFileCsv();  // almacenamiento de datos

            cout << endl << "Press Ctrl+C to exit..." << endl;
            cout << "*******************************" << endl << endl;

            cout << n << endl << endl;
            n++;

            getCurrentTime();

        }
    }
}

/************************************************************************/
void ThreadImpl::saveHomePrajectory(){       /** Save waiter home pose & Initial VEL-ACC **/

    cout << "[configuring] ... STEP 2 " << endl;

    // Obtaining waiter homePoss in Cartesian Space
    if ( ! leftArmIEncoders->getEncoders( iniQ.data() ) )    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;    }
    if ( ! leftArmICartesianSolver->fwdKin(iniQ,curX_AAS) )    {
        CD_ERROR("fwdKin failed.\n");    }
    KinRepresentation::decodePose(curX_AAS, iniX, KinRepresentation::coordinate_system::CARTESIAN, KinRepresentation::orientation_system::AXIS_ANGLE, KinRepresentation::angular_units::DEGREES);

    homeX[0] = iniX[0]; // new X position
    homeX[1] = iniX[1]; // new Y position
    homeX[2] = iniX[2]; // new Z position
    homeX[3] = iniX[3];
    homeX[4] = iniX[4];
    homeX[5] = iniX[5];
    homeX[6] = iniX[6];

    yarp::os::Time::delay(0.5);
    cout << "[success] Waiter Home Pose Saved." << endl;

    //double initspe[7] = {20.0,20.0,20.0,20.0,20.0,20.0,0.0}; // --set NEW ref speed
    std::vector<double> armSpeeds(7,20.0); // 7,30.0
    //leftArmIPositionControl->setRefSpeeds(initspe);
    if(!leftArmIPositionControl->setRefSpeeds(armSpeeds.data())){
        printf("[Error] Problems setting reference speed on left-arm joints.\n");
        return;
    }

    //double initacc[7] = {20.0,20.0,20.0,20.0,20.0,20,0.0}; // --set NEW ref accelaration
    std::vector<double> armAccelerations(7,20.0); // 7,30.0
    //leftArmIPositionControl->setRefAccelerations(initacc);
    if(!leftArmIPositionControl->setRefAccelerations(armAccelerations.data())){
        printf("[Error] Problems setting reference acceleration on left-arm joints.\n");
        return;
    }

    cout << "[success] Ref Speeds and Acc configured." << endl;

    return;
}

/************************************************************************/
void ThreadImpl::openingPorts(){       /** Opening Ports & Connecting with sensor programs **/

    cout << "[configuring] ... STEP 3 " << endl;

    //-- OPEN YARP PORTS
    portImu->open("/objectBal/inertial:i");

    cout << "\n [ATENTION] User must activate sensor programs.\n" << endl;
    yarp::os::Time::delay(5);

    //-- CONNECTIONS PORTS
/*    // ft right foot
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
    yarp::os::Time::delay(0.5);*/
    // imu trunk
    Network::connect("/inertial", "/objectBal/inertial:i");
    if ( NetworkBase::isConnected("/inertial", "/objectBal/inertial:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /objectBal/inertial:i." << endl;
    } else cout << "[success] Connected to IMU." << endl;
    yarp::os::Time::delay(0.5);

    return;
}

/************************************************************************/
void ThreadImpl::checkBottleReady(){       /** Calculating _d parameter **/

    cout << "[waiting bottle] ... STEP 3 " << endl;

    readSensorsFT3();

    if (_LA._F.fx>-5 && f==0){   // botella NO puesta
        cout << "[error] ... botella NO puesta " << endl;
        yarp::os::Time::delay(0.5);
        e=0;        }

    if (_LA._F.fx<-5 && f==0){   // botella puesta
        cout << "[success] ... botella puesta en 3" << endl;
        yarp::os::Time::delay(1);
        cout << "[success] ... botella puesta en 2" << endl;
        yarp::os::Time::delay(1);
        cout << "[success] ... botella puesta en 1" << endl;
        yarp::os::Time::delay(1);
        e=250;        }

}

/************************************************************************/
void ThreadImpl::calcParam_D(){       /** Calculating _d parameter **/

    cout << "[configuring] ... STEP 4 " << endl;

    readSensorsFT3();

/*    // Para promediar el punto de apoyo respecto del centro de la bandeja
    if (f==1){
        _FTLeftHand._initT.my = _FTLeftHand._initT.my /100;
        _off._M.my += + _FTLeftHand._initT.my;
        _FTLeftHand._initT.mx = _FTLeftHand._initT.mx /100;
        _off._M.mx += + _FTLeftHand._initT.mx;
        e++;
        printf(".......................e = %d\n", e);
        printf("......................My = %f\n", _off._M.my);
        printf("......................Mx = %f\n", _off._M.mx);

        if (e==250) {
            _d_X = _off._M.my / e; // distancia en el eje X del JR3 a la botella
            _d_Y = _off._M.mx / e; // distancia en el eje Y del JR3 a la botella
            printf("distancia _d = %f ,\t %f \n", _d_X, _d_Y);
        }
    }*/

}

/************************************************************************/
void ThreadImpl::getInitialTime()
{
    if (n==1){init_time = Time::now();}
    init_loop = Time::now();
    it_time = init_loop - it_prev;
    it_prev = init_loop;
}

/************************************************************************/
void ThreadImpl::readSensorsFT3(){     /** Reading input messages from FT3 SENSORS    **/

    //--- FT-Sensor 3 left arm

    yarp::sig::Vector ch3;
    int ret = ft3AnalogSensor->read(ch3); // lecture from sensor JR3 ch3 - left arm
    if (ret == yarp::dev::IAnalogSensor::AS_OK)
    {
        _LA._F.fx = ch3[0];
        _LA._F.fy = ch3[1];
        _LA._F.fz = ch3[2];
        _LA._T.mx = ch3[3];
        _LA._T.my = ch3[4];
        _LA._T.mz = ch3[5];

        //std::printf("Good read, got: %s\n",ch3.toString().c_str()); // print on terminal vector ch3
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
void ThreadImpl::axesTransform1(){  /** Transformation matrix between TEO_body_axes (world) and Jr3_axes with horizontal tray (waiter)    **/

    _tray._F.fx = + _LA._F.fz;// - _off._F.fz;
    _tray._F.fy = + _LA._F.fy;// - _off._F.fy;
    _tray._F.fz = + _LA._F.fx;// + _off._F.fx;
    _tray._M.mx = + _LA._T.mz;// - _off._M.mz;
    _tray._M.my = + _LA._T.my;// - _off._M.my;
    _tray._M.mz = + _LA._T.mx;// - _off._M.mx;

    _tray._F.fx = _tray._F.fx * 100; // *10 redondeo un decimal // *100 redondeo dos decimales
    _tray._F.fy = _tray._F.fy * 100;
    _tray._F.fz = _tray._F.fz * 100;
    _tray._M.mx = _tray._M.mx * 100;
    _tray._M.my = _tray._M.my * 100;
    _tray._M.mz = _tray._M.mz * 100;

    _tray._F.fx = trunc(_tray._F.fx);
    _tray._F.fy = trunc(_tray._F.fy);
    _tray._F.fz = trunc(_tray._F.fz);
    _tray._M.mx = trunc(_tray._M.mx);
    _tray._M.my = trunc(_tray._M.my);
    _tray._M.mz = trunc(_tray._M.mz);

    _tray._F.fx = _tray._F.fx / 100;
    _tray._F.fy = _tray._F.fy / 100;
    _tray._F.fz = _tray._F.fz / 100;
    _tray._M.mx = _tray._M.mx / 1000;
    _tray._M.my = _tray._M.my / 1000;
    _tray._M.mz = _tray._M.mz / 1000;

    if ((_tray._F.fx<0.1)&&(_tray._F.fx>-0.1)) { //filtro para valores menores de 0.1 N
        _tray._F.fx=0.0;}
    if ((_tray._F.fy<0.1)&&(_tray._F.fy>-0.1)) { //filtro para valores menores de 0.1 N
        _tray._F.fy=0.0;}
    if ((_tray._F.fz<0.1)&&(_tray._F.fz>-0.1)) { //filtro para valores menores de 0.1 N
        _tray._F.fz=0.0;}

}

/************************************************************************/
void ThreadImpl::axesTransform2(){  /** Force/torque Transformation depending on the TCP orientation.   **/

    if ( ! leftArmIEncoders->getEncoders( curQ.data() ) )    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;    }
    if (fabs(curQ[0] - befQ[0]) < 0.2)  {
        curQ[0] = befQ[0];    }
    if (fabs(curQ[1] - befQ[1])<0.2)  {
        curQ[1] = befQ[1];    }
    if (fabs(curQ[2] - befQ[2])<0.2)  {
        curQ[2] = befQ[2];    }
    if (fabs(curQ[3] - befQ[3])<0.5)  {
        curQ[3] = befQ[3];    }
    if (fabs(curQ[4] - befQ[4])<0.5)  {
        curQ[4] = befQ[4];    }
    if (fabs(curQ[5] - befQ[5])<0.5)  {
        curQ[5] = befQ[5];    }
    if (fabs(curQ[6] - befQ[6])<0.5)  {
        curQ[6] = befQ[6];    }


    if ( ! leftArmICartesianSolver->fwdKin(curQ,curX) )    {
        CD_ERROR("fwdKin failed.\n");    }

/*        //normalizando (no hace falta xq ya esta predefinido)
    //_normVector = sqrt(pow((cX[3]),2) + pow((cX[4]),2) + pow((cX[5]),2));
    //cX[3] = cX[3] / _normVector;
    //cX[4] = cX[4] / _normVector;
    //cX[5] = cX[5] / _normVector;

    //redondeando los quaternios
    //int i;
    //for (i=3;i<=6;i++)   {
        //if (cX[i]>0)  {
        //    cX[i] = round (cX[i]); //} // ceil or floor
        //else{    cX[i] = ceil (cX[i]);    }    }

    //convirtiendo de grados a radianes
    //angle= cX[6];*/

    curX[6]=(curX[6]/180)*3.1415926;
    if ((curX[6]<0.015))     {
        curX[3] = 0;
        curX[4] = 0;
        curX[5] = 0;    }
    //convirtiendo en quaternios
    quat[0] = cos(curX[6]/2); // angulo theta
    quat[1] = curX[3] * sin(curX[6]/2); // eje X
    quat[2] = curX[4] * sin(curX[6]/2); // eje Y
    quat[3] = curX[5] * sin(curX[6]/2); // eje Z, siempre a cero

    if (quat[0]==0 && quat[1]==0 && quat[2]==0 && quat[3]==0) {
        quat[1]=1;    }


    //calculando quaternio conjugado
    quatC = quat;
    quatC[1] = - quat[1];
    quatC[2] = - quat[2];
    quatC[2] = - quat[3];

    //transformada      preFinalForce   = quat              *   _initF
    //                  Final_F     = preFF    *   quatC
    preFF[0]=( - (quat[1]*_tray._F.fx) - (quat[2]*_tray._F.fy) - (quat[3]*_tray._F.fz));
    preFF[1]=((quat[0]*_tray._F.fx) + (quat[2]*_tray._F.fz) - (quat[3]*_tray._F.fy));
    preFF[2]=((quat[0]*_tray._F.fy) - (quat[1]*_tray._F.fz) + (quat[3]*_tray._F.fx));
    preFF[3]=((quat[0]*_tray._F.fz) + (quat[1]*_tray._F.fy) - (quat[2]*_tray._F.fx));
    FF[0]=((preFF[0]*quatC[0]) - (preFF[1]*quatC[1]) - (preFF[2]*quatC[2])) - (preFF[3]*quatC[3]);
    FF[1]=((preFF[0]*quatC[1]) + (preFF[1]*quatC[0]) + (preFF[2]*quatC[3])) - (preFF[3]*quatC[2]); //fx
    FF[2]=((preFF[0]*quatC[2]) - (preFF[1]*quatC[3]) + (preFF[2]*quatC[0])) + (preFF[3]*quatC[1]); //fy
    FF[3]=((preFF[0]*quatC[3]) + (preFF[1]*quatC[2]) - (preFF[2]*quatC[1])) + (preFF[3]*quatC[0]); //fz

    //FF[0] = trunc(FF[0]);
    //FF[1] = trunc(FF[1]);
    //FF[2] = trunc(FF[2]);
    //FF[3] = trunc(FF[3]);

    //transformada      preFM   = quat              *   _initM
    //                  FM     = _preFM    *   quatC
    //preFM[0]=((-quat[1]*_tray._M.mx) - (quat[2]*_tray._M.my) - (quat[3]*_tray._M.mz));
    //preFM[1]=((quat[0]*_tray._M.mx) + (quat[2]*_tray._M.mz) - (quat[3]*_tray._M.my));
    //preFM[2]=((quat[0]*_tray._M.my) - (quat[1]*_tray._M.mz) + (quat[3]*_tray._M.mx));
    //preFM[3]=((quat[0]*_tray._M.mz) + (quat[1]*_tray._M.my) - (quat[2]*_tray._M.mx));
    //FM[0]=((preFM[0]*quatC[0]) - (preFM[1]*quatC[1]) - (preFM[2]*quatC[2])) - (preFM[3]*quatC[3]);
    //FM[1]=((preFM[0]*quatC[1]) + (preFM[1]*quatC[0]) + (preFM[2]*quatC[3])) - (preFM[3]*quatC[2]); //mx
    //FM[2]=((preFM[0]*quatC[2]) - (preFM[1]*quatC[3]) + (preFM[2]*quatC[0])) + (preFM[3]*quatC[1]); //my
    //FM[3]=((preFM[0]*quatC[3]) + (preFM[1]*quatC[2]) - (preFM[2]*quatC[1])) + (preFM[3]*quatC[0]); //mz

}

/************************************************************************/
void ThreadImpl::zmpComp(){         /** Bottle ZMP measurements    **/


    if ( ! leftArmIEncoders->getEncoders( curQ.data() ) )    { //obtencion de los valores articulares (encoders absolutos)
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;    }

    if ( ! leftArmICartesianSolver->fwdKin(curQ,curX_AAS) )    {
        CD_ERROR("fwdKin failed.\n");    }
    KinRepresentation::decodePose(curX_AAS, curX, KinRepresentation::coordinate_system::CARTESIAN, KinRepresentation::orientation_system::AXIS_ANGLE, KinRepresentation::angular_units::DEGREES);


    //angle=(cX[6]/180)*3.1415926; // transformando a rad
/*
    _thetaX = -(atan((fabs(FF[3])/FF[1]))); //sobre el plano YZ en rad
    _thetaY = (atan((fabs(FF[3])/FF[2]))); //sobre el plano XZ en rad
    if (_thetaX < 0)    {
        _thetaX += + 3.1415926/2;}
    else    {
        _thetaX -= + 3.1415926/2;}
    if (_thetaY < 0)    {
        _thetaY += + 3.1415926/2;}
    else    {
        _thetaY -= + 3.1415926/2;}
 */
    _thetaXX = (atan(_tray._F.fx/(fabs(_tray._F.fz)))); //sobre el plano Y en rad
    _thetaYY = -(atan(_tray._F.fy/(fabs(_tray._F.fz)))); //sobre el plano X en rad

    _thetaX = (((atan(FF[1]/fabs(FF[3])))*180)/3.1415926); //sobre el plano Y en grados
    _thetaY = -(((atan(FF[2]/fabs(FF[3])))*180)/3.1415926); //sobre el plano X en grados
/*    if (_thetaX < 0)    {
        _thetaX += + 90;}
    else    {
        _thetaX -= + 90;}
    if (_thetaY < 0)    {
        _thetaY += + 90;}
    else    {
        _thetaY -= + 90;}*/

/*    _thetaX = -(atan((fabs(_tray._F.fz)/_tray._F.fx))); //sobre el plano YZ en rad
    _thetaY = (atan((fabs(_tray._F.fz)/_tray._F.fy))); //sobre el plano XZ en rad
    if (_thetaX < 0)    {
        _thetaX += + 3.1415926/2;}
    else    {
        _thetaX -= + 3.1415926/2;}
    if (_thetaY < 0)    {
        _thetaY += + 3.1415926/2;}
    else    {
        _thetaY -= + 3.1415926/2;}*/

/*    if (_tray._F.fz==0)    {
        _tray._zmp.x_zmp = _tray._zmp.x_zmp; // Metros
        _tray._zmp.y_zmp = _tray._zmp.y_zmp; // Metros
    }else{
        _tray._zmp.x_zmp = ( - _tray._M.my / ((_tray._F.fz)*cos(_thetaX)) - (_l*sin(_thetaX)) - _d); // Metros
        _tray._zmp.y_zmp = ( _tray._M.mx / ((_tray._F.fz)*cos(_thetaY)) - (_l*sin(_thetaY)) );}*/ // Metros

    // testeando****************************************************
    if (_tray._F.fz==0)    {
        _tray._zmp.x_zmp = _tray._zmp.x_zmp; // Metros
        _tray._zmp.y_zmp = _tray._zmp.y_zmp; // Metros
    }else{
        _tray._zmp.x_zmp = ((- _tray._M.my / (-9.72*cos(_thetaXX))) - (_l*FF[1]/(-9.72*cos(_thetaXX)) ) - _d_X); // Metros
        _tray._zmp.y_zmp = ((_tray._M.mx / (-9.72*cos(_thetaYY)))   - (_l*FF[2]/(-9.72*cos(_thetaYY)) ) ); // Metros
        _tray._zmp.x_zmp = 1000 * _tray._zmp.x_zmp; // conversion a milimetros
        _tray._zmp.y_zmp = 1000 * _tray._zmp.y_zmp; // conversion a milimetros

        /*_tray._zmp.x_zmp = trunc(_tray._zmp.x_zmp);
        _tray._zmp.y_zmp = trunc(_tray._zmp.y_zmp);*/

       // _tray._zmp.x_zmp = ( - _tray._M.my / ((FF[3])*cos(_thetaX)) - ((_l*FF[1]*tan(_thetaX))/FF[3]) ); // Metros
       // _tray._zmp.y_zmp = ( _tray._M.mx / ((FF[3])*cos(_thetaY)) - ((_l*FF[1]*tan(_thetaY))/FF[3]) ); // Metros
    }
    //**************************************************************

/*    _tray._zmp.x_zmp = _tray._zmp.x_zmp*cos(_thetaX);
    _tray._zmp.y_zmp = _tray._zmp.y_zmp*cos(_thetaY);*/

/*    if (_tray._zmp.x_zmp>0.075){ //limitando el maximo ZMP en X positivo
        _tray._zmp.x_zmp = 0.075;
    }if (_tray._zmp.x_zmp<-0.075)    {//limitando el maximo ZMP en X negativo
        _tray._zmp.x_zmp = -0.075;
    }*/

/*    if (_tray._zmp.y_zmp>0.075)    {//limitando el maximo ZMP en Y positivo
        _tray._zmp.y_zmp = 0.075;
    }if (_tray._zmp.y_zmp<-0.075)    {//limitando el maximo ZMP en Y negativo
        _tray._zmp.y_zmp = -0.075;
    }*/


 /*   if ((_tray._zmp.x_zmp<0.001) && (_tray._zmp.x_zmp>-0.001))    {
        _tray._zmp.x_zmp = 0;} //limitando el minimo ZMP en X
    if ((_tray._zmp.y_zmp<0.001) && (_tray._zmp.y_zmp>-0.001))    {
        _tray._zmp.y_zmp = 0;} //limitando el minimo ZMP en Y positivo
*/

    _rzmp = sqrt(pow(_tray._zmp.x_zmp,2) + pow(_tray._zmp.y_zmp,2));

}

/************************************************************************/
void ThreadImpl::LIPM3d(){          /** Control - Joint Position Calculus    **/

    //Generacion de la actuacion a los motores (CONTROL)

    if (_LA._F.fx<-5){   // botella SI puesta

        // version 2
        if (_rzmp>=15)  { // ZMP botella INESTABLE

            desX[3] = curX[3];
            desX[4] = curX[4];
            desX[5] = curX[5];

            _rFxy = sqrt(pow(FF[1],2) + pow(FF[2],2)); // composicion de Fx y Fy
            _alpha = ((atan(_rFxy/(-FF[3])))*180)/(3.1415926); // angulo botella

            desX[6] = (fabs(curX[6]) - (_alpha));

            KinRepresentation::encodePose(desX, desX_AAS, KinRepresentation::coordinate_system::CARTESIAN, KinRepresentation::orientation_system::AXIS_ANGLE, KinRepresentation::angular_units::DEGREES);
            if ( ! leftArmICartesianSolver->invKin(desX_AAS,curQ,desQ) )    {
                CD_ERROR("invKin failed.\n");    }
            if( ! leftArmIPositionControl->positionMove( desQ.data() )) {
                CD_WARNING("setPositions failed, not updating control this iteration.\n");      }

            printf("MOVIENDOME HACIA ...\n");
            befX = desX;
            befQ = desQ;
            _rzmp_b = _rzmp;
        }
        if (_rzmp<15)   { // ZMP botella estable

            if (_rzmp_b<15)   { // me voy a la waiter pose

                desX = iniX;
                KinRepresentation::encodePose(desX, desX_AAS, KinRepresentation::coordinate_system::CARTESIAN, KinRepresentation::orientation_system::AXIS_ANGLE, KinRepresentation::angular_units::DEGREES);
                if ( ! leftArmICartesianSolver->invKin(desX_AAS,iniQ,desQ) )    {
                    CD_ERROR("invKin failed.\n");    }
                if( ! leftArmIPositionControl->positionMove( desQ.data() )) {
                    CD_WARNING("setPositions failed, not updating control this iteration.\n");    }

                printf("WAITER POSE \n");
                befQ = desQ;
                _rzmp_b = _rzmp;
            }
            if (_rzmp_b>15)   { // me quedo donde estoy
                if ( ! leftArmIEncoders->getEncoders( curQ.data() ) )    {
                    CD_WARNING("getEncoders failed, not updating control this iteration.\n");
                    return;    }
                if( ! leftArmIPositionControl->positionMove( curQ.data() )) {
                    CD_WARNING("setPositions failed, not updating control this iteration.\n");      }

                printf("QUIETOOOOOOOOO \n");
                befQ = curQ;
                _rzmp_b = _rzmp;
            }
        }

    /*    // version 1
        if (fabs(_rzmp - _rzmp_b < 40))   {
            if (_rzmp>=10)  { // será necesario programar los limites en los ejes X e Y.
                // calculo del vector unitario de rotacion
                desX[3] = curX[3];
                desX[4] = curX[4];
                desX[5] = curX[5];

                _rFxy = sqrt(pow(FF[1],2) + pow(FF[2],2)); // composicion de Fx y Fy
                _alpha = ((atan(_rFxy/(-FF[3])))*180)/(3.1415926);

                desX[6] = -(fabs(curX[6])+ (_alpha/4));

                KinRepresentation::encodePose(desX, desX_AAS, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES);
                if ( ! leftArmICartesianSolver->invKin(desX_AAS,curQ,desQ) )    {
                    CD_ERROR("invKin failed.\n");    }

                if( ! leftArmIPositionControl->positionMove( desQ.data() )) {
                    CD_WARNING("setPositions failed, not updating control this iteration.\n");      }

                 // calculo del vector unitario de rotacion
              //  desX[3] = -_tray._zmp.y_zmp / _rzmp;
              //  desX[4] = _tray._zmp.x_zmp / _rzmp;
              //  desX[5] = 0;
                // calculo del angulo de rotacion
              //  _rFxy = sqrt(pow(FF[1],2) + pow(FF[2],2)); // composicion de Fx y Fy
              //  if (desX[3]*desX[4]>=0)
              //      desX[6] = (((atan(_rFxy/(-FF[3])))*180)/(3.1415926));
              //  if (desX[3]*desX[4]<0)
              //      desX[6] = -(((atan(_rFxy/(-FF[3])))*180)/(3.1415926));
              //  desX[6] = (desX[6] - curX[6]); /// 2;
                printf("MOVIENDOME HACIA ...\n");
                befX = desX;
                _rzmp_b = _rzmp;
            }

            if (_rzmp<10)   { // será necesario programar los limites en los ejes X e Y.

                if ( ! leftArmIEncoders->getEncoders( curQ.data() ) )    { //obtencion de los valores articulares (encoders absolutos)
                    CD_WARNING("getEncoders failed, not updating control this iteration.\n");
                    return;    }
                if( ! leftArmIPositionControl->positionMove( curQ.data() )) {
                    CD_WARNING("setPositions failed, not updating control this iteration.\n");      }

                printf("THE BOTTLE IS IN EQUILIBRIUM \n");
                befQ = desQ;
                _rzmp_b = _rzmp;
            }
        }
        if (fabs(_rzmp - _rzmp_b > 40))   {

            if ( ! leftArmIEncoders->getEncoders( curQ.data() ) )    { //obtencion de los valores articulares (encoders absolutos)
                CD_WARNING("getEncoders failed, not updating control this iteration.\n");
                return;    }
            if( ! leftArmIPositionControl->positionMove( curQ.data() )) {
                CD_WARNING("setPositions failed, not updating control this iteration.\n");      }

            printf("ME HE PASADO TRES PUEBLOS \n");
            //return;
            befQ = desQ;
            _rzmp_b = _rzmp;
        }
    */
    }

    if (_LA._F.fx>-5){   // botella NO puesta

        cout << "[error] ... botella NO puesta " << endl;

        KinRepresentation::encodePose(iniX, desX_AAS, KinRepresentation::coordinate_system::CARTESIAN, KinRepresentation::orientation_system::AXIS_ANGLE, KinRepresentation::angular_units::DEGREES);
        if ( ! leftArmICartesianSolver->invKin(desX_AAS,befQ,desQ) )    {
            CD_ERROR("invKin failed.\n");    }
        if( ! leftArmIPositionControl->positionMove( desQ.data() )) {
            CD_WARNING("setPositions failed, not updating control this iteration.\n");      }

        yarp::os::Time::delay(1);
    }   

/*  // como se genera la InvKin para actuar en posicion
 *  KinRepresentation::encodePose(desX, desX_AAS, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES);
    if ( ! leftArmICartesianSolver->invKin(desX_AAS,befQ,desQ) )    {
        CD_ERROR("invKin failed.\n");    }

    if( ! leftArmIPositionControl->positionMove( desQ.data() )) {
        CD_WARNING("setPositions failed, not updating control this iteration.\n");    }*/

    return;

}

/************************************************************************/
void ThreadImpl::printData()
{

//    _modFS = sqrt(pow((_tray._F.fx),2) + pow((_tray._F.fy),2) + pow((_tray._F.fz),2));
//    _modFF = sqrt(pow((FF[1]),2) + pow((FF[2]),2) + pow((FF[3]),2));

    cout << "angle: [" << curX[6] << "\t, " << _alpha << "\t, " << desX[6] << "\t " << "]" << endl;//<< FF[1] << "]" << endl;

//    cout << "CurX: [" << cX[3] << "\t, " << cX[4] << "\t, " << cX[5] << "\t, " << cX[6] << "]" << endl;
//    cout << "Quat: [" << quatC[0] << "\t, " << quatC[1] << "\t, " << quatC[2] << "\t, " << quatC[3] << "]" << endl;
    cout << "F_init: [" << _tray._F.fx << "\t, " << _tray._F.fy << "\t, " << _tray._F.fz << "\t " << "]" << endl;//<< FF[1] << "]" << endl;
//    cout << "F_finl: [" << FF[1] << "\t, " << FF[2] << "\t, " << FF[3] << "\t " << "]" << endl;//<< FF[1] << "]" << endl;
    cout << "torque: [" << _tray._M.mx << "\t, " << _tray._M.my << "\t, " << _tray._M.mz << "\t " << "]" << endl;//<< FF[1] << "]" << endl;

//    cout << "F_X: [" << _tray._F.fx << "\t, " << FF[1] << "]" << endl;//<< FF[1] << "]" << endl;
//    cout << "F_Y: [" << _tray._F.fy << "\t, " << FF[2] << "]" << endl;//<< FF[2] << "]" << endl;
//    cout << "F_Z: [" << _tray._F.fz << "\t, " << FF[3] << "]" << endl;//<< FF[3] << "]" << endl;
    //cout << endl << "El angulo imu es: " << _tray._zmp.x_zmp << endl;
    cout << "ZMP: [" << _tray._zmp.x_zmp << "\t, " << _tray._zmp.y_zmp << "]" << endl;
    //cout << "inputAngle: [" << inputAngle << "]" << endl;

    //    cout << "mod: [" << _modFS << "\t, " << _modFF << "]" << endl;
//    cout << "the1: [" << _thetaXX << "\t, " << _thetaYY << "]" << endl;
//    cout << "the2: [" << _thetaX << "\t, " << _thetaY << "]" << endl;

//    cout << "CurX: [" << cX[3] << "\t, " << cX[4] << "\t, " << cX[5] << "\t, " << cX[6] << "]" << endl;
//    cout << "DesX: [" << dX[0] << "\t, " << dX[1] << "\t, " << dX[2] << "]" << endl;
//    cout << "DesX: [" << dX[3] << "\t, " << dX[4] << "\t, " << dX[5] << "\t, " << dX[6] << "]" << endl;
    cout << "CurX: [" << curX[0] << "\t, " << curX[1] << "\t, " << curX[2] << "\t, " << curX[3] << "\t, " << curX[4] << "\t, " << curX[5] << "\t, " << curX[6] << "]" << endl;
    cout << "DesX: [" << desX[0] << "\t, " << desX[1] << "\t, " << desX[2] << "\t, " << desX[3] << "\t, " << desX[4] << "\t, " << desX[5] << "\t, " << desX[6] << "]" << endl;
    cout << "CurQ: [" << curQ[0] << "\t, " << curQ[1] << "\t, " << curQ[2] << "\t, " << curQ[3] << "\t, " << curQ[4] << "\t, " << curQ[5] << "]" << endl;
    cout << "DesQ: [" << desQ[0] << "\t, " << desQ[1] << "\t, " << desQ[2] << "\t, " << desQ[3] << "\t, " << desQ[4] << "\t, " << desQ[5] << "]" << endl;
//    cout << "befQ: [" << beforeQ[0] << "\t, " << beforeQ[1] << "\t, " << beforeQ[2] << "\t, " << beforeQ[3] << "\t, " << beforeQ[4] << "\t, " << beforeQ[5] << "\t, " << beforeQ[6] << "]" << endl;

}

/************************************************************************/
void ThreadImpl::confCSVfile(){       /** Configuring CSV file    **/

    cout << "[configuring] ... STEP 1 " << endl;
    fp = fopen("../data_3dslope_threat.csv","w+");
    fprintf(fp,"act_time,fx,fy,fz,Mx,My,Mz,Ang_imu,Acc_imu,Xzmp,Yzmp,CX3,CX4,CX5,CX6,TX1,TY1,TX2,TY2,iter");
    yarp::os::Time::delay(1);

    cout << "[success] data_3dslope_threat.csv file configured." << endl;
}

/************************************************************************/
void ThreadImpl::saveInFileCsv()
{
    fprintf(fp,"\n%.4f", act_time);

    fprintf(fp,",%.10f", _LA._F.fx);
    fprintf(fp,",%.10f", _LA._F.fy);
    fprintf(fp,",%.10f", _LA._F.fz);
    fprintf(fp,",%.10f", _LA._T.mx);
    fprintf(fp,",%.10f", _LA._T.my);
    fprintf(fp,",%.10f", _LA._T.mz);

    fprintf(fp,",%.10f", ang_x); // angle X imu
    fprintf(fp,",%.10f", acc_x); // acceleration X imu

    fprintf(fp,",%.8f", _tray._zmp.x_zmp);
    fprintf(fp,",%.8f", _tray._zmp.y_zmp);

    fprintf(fp,",%.4f", curX[3]);
    fprintf(fp,",%.4f", curX[4]);
    fprintf(fp,",%.4f", curX[5]);
    fprintf(fp,",%.4f", curX[6]);

    fprintf(fp,",%.4f", _thetaXX);
    fprintf(fp,",%.4f", _thetaYY);
    fprintf(fp,",%.4f", _thetaX);
    fprintf(fp,",%.4f", _thetaY);

    fprintf(fp,",%i", n);
}

/************************************************************************/
void ThreadImpl::getCurrentTime()
{
    act_time = Time::now() - init_time;
    act_loop = Time::now() - init_loop;
}

/************************************************************************/

}   // namespace roboticslab
