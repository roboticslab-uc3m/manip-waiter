// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ThreadImpl.hpp"

namespace roboticslab
{
/************************************************************************/
bool ThreadImpl::threadInit() {
    printf("[success] entrando en ratethread -> init/run\n");

    a = 0;
    b = 0;
    w = 0;
    n = 1;

    inputAngle = -9; // initial angle pose
    _d = 0.025; // distance in metres
    _l = 0.05;  // distance in metres

    quat.resize(4);
    quatC.resize(4);
    preFF.resize(4);
    FF.resize(4);

    _rzmp = 0; // initial
    _rzmp_b = 0;
    x_zmp_b = 0;
    y_zmp_b = 0;

    //preFM.resize(4);
    //FM.resize(4); // quaternios

    _off._F.fx = 0;
    _off._F.fy = 0;
    _off._F.fz = 0;
    _off._M.mx = 0;
    _off._M.my = 0;
    _off._M.mz = 0;

    curQ.resize(7);
    curX.resize(7);
    curX_AAS.resize(6);
    desX.resize(7);
    desX_AAS.resize(6);
    desQ.resize(7);
    befQ.resize(7);

    _FTLeftHand._initF.fx = 0;
    _FTLeftHand._initF.fy = 0;
    _FTLeftHand._initF.fz = 0;
    _FTLeftHand._initT.mx = 0;
    _FTLeftHand._initT.my = 0;
    _FTLeftHand._initT.mz = 0;
return true;
}

/************************************************************************/
void ThreadImpl::run()
{
    while(!isStopping()) {

        if (b!=250)    {    // STEP 1 - deleting sensor offset
            offSetJR3();
        }
        if (a==0 && b==250)    {    // STEP 2 - moving the arm to the stability pose
            preprogrammedInitTrajectory();
            //iPositionControl->setPositionMode();
            a=1;    }


        if (b==250 && a==1 && inputAngle<12)   {     // STEP 3 - main code
            getInitialTime();

            readSensors();

            axesTransform1();
            axesTransform2();

            zmpComp(); // calculo del ZMP_FT
            LIPM3d(); // Calculo y generacion de la actuacion en funcion del ZMP_tray

            printData();
            cout << endl << "Press ENTER to exit..." << endl;
            cout << "*******************************" << endl << endl;
            saveInFileCsv();  // almacenamiento de datos
            n++;
            cout << n << endl << endl;

            getCurrentTime();

        }

    }

}

/************************************************************************/
void ThreadImpl::offSetJR3(){       /** Offset JR3 measurements    **/

    readSensors();
    _off._F.fx += -9.72 - _FTLeftHand._initF.fx; //9.72N = 0.99kg * 9.81mm/s2
    _off._F.fy += + _FTLeftHand._initF.fy;
    _off._F.fz += + _FTLeftHand._initF.fz;
    _off._M.mx += + _FTLeftHand._initT.mx;
//    _off._M.my += 0.243 - _jr3._initT.my; //0.243Nm = 9.72N * 0.025m
    _off._M.my += + _FTLeftHand._initT.my; //testeando *************
    _off._M.mz += + _FTLeftHand._initT.mz;

    b++;
    printf(".......................b = %d\n", b);

    if (b==250) {
        _off._F.fx = _off._F.fx / b;
        _off._F.fy = _off._F.fy / b;
        _off._F.fz = _off._F.fz / b;
        _off._M.mx = _off._M.mx / b;
        _off._M.my = _off._M.my / b;
        _off._M.mz = _off._M.mz / b;
    }

}

/************************************************************************/
void ThreadImpl::preprogrammedInitTrajectory()
{
    fp = fopen("../data_testCap.csv","w+");

    /*leftArmIEncoders->getAxes(&numLeftArmJoints);
    CD_INFO("numRobotJoints: %d.\n",numLeftArmJoints);*/

    double initpos[7] = {-30,-1,0,-90,0,30,0};
    leftArmIPositionControl2->positionMove(initpos);
    yarp::os::Time::delay(15);
    printf("...........ESTOY AQUI............\n");

/** ---- designate initial position --------------- **/ //comprobar funcionalidad
    if ( ! leftArmIEncoders->getEncoders( befQ.data() ) )    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;    }
    /** --------------------------------------------------- **/

    if ( ! leftArmICartesianSolver->fwdKin(befQ,curX_AAS) )    {
        CD_ERROR("fwdKin failed.\n");    }
    KinRepresentation::decodePose(curX_AAS, befX, KinRepresentation::coordinate_system::CARTESIAN, KinRepresentation::orientation_system::AXIS_ANGLE, KinRepresentation::angular_units::DEGREES);

    /** ----- generate initial POSE --------------- **/
    // -- 0.303928 0.347326 0.248109 0.008763 -0.999962 -0.000286 10.03554
    desX[0] = befX[0]; // new X position
    desX[1] = befX[1]; // new Y position
    desX[2] = befX[2]; // new Z position
    desX[3]= 1; // a: 1 - b: 0 - c: 1
    desX[4]= 1; // a: 0 - b: 1 - c: 1
    desX[5]= 0;
    desX[6]= -10;
    KinRepresentation::encodePose(desX, desX_AAS, KinRepresentation::coordinate_system::CARTESIAN, KinRepresentation::orientation_system::AXIS_ANGLE, KinRepresentation::angular_units::DEGREES);


    if ( ! leftArmICartesianSolver->invKin(desX_AAS,befQ,desQ) )    {
        CD_ERROR("invKin failed.\n");    }

    if( ! leftArmIPositionControl2->positionMove( desQ.data() )) {
        CD_WARNING("setPositions failed, not updating control this iteration.\n");    }

    yarp::os::Time::delay(5);
    printf("end MOVE TO START POSITION\n");

/** ---- designate initial position --------------- **/
    if ( ! leftArmIEncoders->getEncoders( befQ.data() ) )    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;    }

    double initspe[7] = {10.0,10.0,10.0,10.0,10.0,10.0,0.0}; // --set NEW ref speed
    leftArmIPositionControl2->setRefSpeeds(initspe);
    double initacc[7] = {10.0,10.0,10.0,10.0,10.0,10,0.0}; // --set NEW ref accelaration
    leftArmIPositionControl2->setRefAccelerations(initacc);


    fprintf(fp,"fx,fy,fz,Mx,My,Mz,Xzmp,Yzmp,CX3,CX4,CX5,CX6,TX1,TY1,TX2,TY2,act_time");
    yarp::os::Time::delay(10);
    return;
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
void ThreadImpl::readSensors(){     /** Reading input messages from SENSORS    **/

/*        //--- FT-Sensor 0 right leg
    Bottle ch0;
    portFt0->read(ch0); // lectura del sensor JR3 ch0 - right foot
    _FTRightFoot._initF.fx = ch0.get(0).asDouble();
    _FTRightFoot._initF.fy = ch0.get(1).asDouble();
    _FTRightFoot._initF.fz = ch0.get(2).asDouble();
    _FTRightFoot._initT.mx = ch0.get(3).asDouble();
    _FTRightFoot._initT.my = ch0.get(4).asDouble();
    _FTRightFoot._initT.mz = ch0.get(5).asDouble();*/

/*        //--- FT-Sensor 1 left leg
    Bottle ch1;
    portFt1->read(ch1); // lectura del sensor JR3 ch1 - left foot
    _FTLeftFoot._initF.fx = ch1.get(0).asDouble();
    _FTLeftFoot._initF.fy = ch1.get(1).asDouble();
    _FTLeftFoot._initF.fz = ch1.get(2).asDouble();
    _FTLeftFoot._initT.mx = ch1.get(3).asDouble();
    _FTLeftFoot._initT.my = ch1.get(4).asDouble();
    _FTLeftFoot._initT.mz = ch1.get(5).asDouble();*/

/*        //--- FT-Sensor 2 right hand
    Bottle ch2;
    portFt2->read(ch2); // lectura del sensor JR3 ch2 - right hand
    _FTRightHand._initF.fx = ch2.get(0).asDouble();
    _FTRightHand._initF.fy = ch2.get(1).asDouble();
    _FTRightHand._initF.fx = ch2.get(2).asDouble();
    _FTRightHand._initT.mx = ch2.get(3).asDouble();
    _FTRightHand._initT.my = ch2.get(4).asDouble();
    _FTRightHand._initT.mz = ch2.get(5).asDouble();*/

    //--- FT-Sensor 3 left hand
    Bottle ch3;
    portFt3->read(ch3); // lectura del sensor JR3 ch3 - left hand
    _FTLeftHand._initF.fx = ch3.get(0).asDouble();
    _FTLeftHand._initF.fy = ch3.get(1).asDouble();
    _FTLeftHand._initF.fz = ch3.get(2).asDouble();
    _FTLeftHand._initT.mx = ch3.get(3).asDouble();
    _FTLeftHand._initT.my = ch3.get(4).asDouble();
    _FTLeftHand._initT.mz = ch3.get(5).asDouble();

/*        //--- Inertial-Sensor
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
     ddz_robot = ddz;*/

}

/************************************************************************/
void ThreadImpl::axesTransform1(){  /** Transformation matrix between TEO_body_axes (world) and Jr3_axes with horizontal tray (waiter)    **/

    _tray._F.fx = + _FTLeftHand._initF.fz - _off._F.fz;
    _tray._F.fy = + _FTLeftHand._initF.fy - _off._F.fy;
    _tray._F.fz = + _FTLeftHand._initF.fx + _off._F.fx;
    _tray._M.mx = + _FTLeftHand._initT.mz - _off._M.mz;
    _tray._M.my = + _FTLeftHand._initT.my - _off._M.my;
    _tray._M.mz = + _FTLeftHand._initT.mx - _off._M.mx;

    _tray._F.fx = _tray._F.fx * 10;
    _tray._F.fy = _tray._F.fy * 10;
    _tray._F.fz = _tray._F.fz * 10;
    _tray._M.mx = _tray._M.mx * 10;
    _tray._M.my = _tray._M.my * 10;
    _tray._M.mz = _tray._M.mz * 10;

    _tray._F.fx = trunc(_tray._F.fx);
    _tray._F.fy = trunc(_tray._F.fy);
    _tray._F.fz = trunc(_tray._F.fz);
    _tray._M.mx = trunc(_tray._M.mx);
    _tray._M.my = trunc(_tray._M.my);
    _tray._M.mz = trunc(_tray._M.mz);
/*
    _tray._F.fx = _tray._F.fx / 10;
    _tray._F.fy = _tray._F.fy / 10;
    _tray._F.fz = _tray._F.fz / 10;
    _tray._M.mx = _tray._M.mx / 100;
    _tray._M.my = _tray._M.my / 100;
    _tray._M.mz = _tray._M.mz / 100;
*/
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

    FF[0] = trunc(FF[0]);
    FF[1] = trunc(FF[1]);
    FF[2] = trunc(FF[2]);
    FF[3] = trunc(FF[3]);

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

    /*
    if ( ! iEncoders->getEncoders( cQ.data() ) )    { //obtencion de los valores articulares (encoders absolutos)
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        std::cout << "ZMP: [dentro]" << endl;
        return;    }
*/
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
    _thetaXX = (atan(_tray._F.fx/(fabs(_tray._F.fz)))); //sobre el plano YZ en rad
    _thetaYY = -(atan(_tray._F.fy/(fabs(_tray._F.fz)))); //sobre el plano XZ en rad

    _thetaX = (((atan(FF[1]/fabs(FF[3])))*180)/3.1415926); //sobre el plano YZ en grados
    _thetaY = -(((atan(FF[2]/fabs(FF[3])))*180)/3.1415926); //sobre el plano XZ en grados
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
        _tray._zmp.x_zmp = ((- _tray._M.my / (-972*cos(_thetaXX))) - (_l*FF[1]/(-97.2*cos(_thetaXX)) )); // Metros
        _tray._zmp.y_zmp = ((_tray._M.mx / (-972*cos(_thetaYY)))   - (_l*FF[2]/(-97.2*cos(_thetaYY)) )); // Metros
        _tray._zmp.x_zmp = 1000 * _tray._zmp.x_zmp;
        _tray._zmp.y_zmp = 1000 * _tray._zmp.y_zmp;
        _tray._zmp.x_zmp = trunc(_tray._zmp.x_zmp);
        _tray._zmp.y_zmp = trunc(_tray._zmp.y_zmp);
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
    }*/if ((_tray._zmp.x_zmp<0.001) && (_tray._zmp.x_zmp>-0.001))    {
        _tray._zmp.x_zmp = 0;} //limitando el minimo ZMP en X

/*    if (_tray._zmp.y_zmp>0.075)    {//limitando el maximo ZMP en Y positivo
        _tray._zmp.y_zmp = 0.075;
    }if (_tray._zmp.y_zmp<-0.075)    {//limitando el maximo ZMP en Y negativo
        _tray._zmp.y_zmp = -0.075;
    }*/if ((_tray._zmp.y_zmp<0.001) && (_tray._zmp.y_zmp>-0.001))    {
        _tray._zmp.y_zmp = 0;} //limitando el minimo ZMP en Y positivo

    _rzmp = sqrt(pow(_tray._zmp.x_zmp,2) + pow(_tray._zmp.y_zmp,2));

}

/************************************************************************/
void ThreadImpl::LIPM3d(){          /** Control - Joint Position Calculus    **/

    //desX[0] = curX[0]; // new X position
    //desX[1] = curX[1]; // new Y position
    //desX[2] = curX[2]; // new Z position
    // ref[x,y,z]: 0.302982 0.34692 0.247723
    //dX[0] = 0.302982; // new X position
    //dX[1] = 0.34692; // new Y position
    //dX[2] = 0.247723; // new Z position

    if (w<100)  {
        w++;
        printf("\n %d \n",w);
        return;
    }
    else    {
        w=0;
        desX[3]= 1; // a: 1 - b: 0 - c: 1
        desX[4]= 1; // a: 0 - b: 1 - c: 1
        desX[5]= 0;
        desX[6]= inputAngle;
        inputAngle++;
        KinRepresentation::encodePose(desX, desX_AAS, KinRepresentation::coordinate_system::CARTESIAN, KinRepresentation::orientation_system::AXIS_ANGLE, KinRepresentation::angular_units::DEGREES);
    }

    if ( ! leftArmICartesianSolver->invKin(desX_AAS,curQ,desQ) )    {
        CD_ERROR("invKin failed.\n");    }

    if( ! leftArmIPositionControl2->positionMove( desQ.data() )) {
        CD_WARNING("setPositions failed, not updating control this iteration.\n");    }

//    yarp::os::Time::delay(3);
    return;

}

/************************************************************************/
void ThreadImpl::printData()
{

//    _modFS = sqrt(pow((_tray._F.fx),2) + pow((_tray._F.fy),2) + pow((_tray._F.fz),2));
//    _modFF = sqrt(pow((FF[1]),2) + pow((FF[2]),2) + pow((FF[3]),2));

//    cout << "CurX: [" << cX[3] << "\t, " << cX[4] << "\t, " << cX[5] << "\t, " << cX[6] << "]" << endl;
//    cout << "Quat: [" << quatC[0] << "\t, " << quatC[1] << "\t, " << quatC[2] << "\t, " << quatC[3] << "]" << endl;
//    cout << "F_init: [" << _tray._F.fx << "\t, " << _tray._F.fy << "\t, " << _tray._F.fz << "\t " << "]" << endl;//<< FF[1] << "]" << endl;
//    cout << "F_finl: [" << FF[1] << "\t, " << FF[2] << "\t, " << FF[3] << "\t " << "]" << endl;//<< FF[1] << "]" << endl;
//    cout << "torque: [" << _tray._M.mx << "\t, " << _tray._M.my << "\t, " << _tray._M.mz << "\t " << "]" << endl;//<< FF[1] << "]" << endl;

//    cout << "F_X: [" << _tray._F.fx << "\t, " << FF[1] << "]" << endl;//<< FF[1] << "]" << endl;
//    cout << "F_Y: [" << _tray._F.fy << "\t, " << FF[2] << "]" << endl;//<< FF[2] << "]" << endl;
//    cout << "F_Z: [" << _tray._F.fz << "\t, " << FF[3] << "]" << endl;//<< FF[3] << "]" << endl;
    //cout << endl << "El angulo imu es: " << _tray._zmp.x_zmp << endl;
    cout << "ZMP: [" << _tray._zmp.x_zmp << "\t, " << _tray._zmp.y_zmp << "]" << endl;
    cout << "inputAngle: [" << inputAngle << "]" << endl;

    //    cout << "mod: [" << _modFS << "\t, " << _modFF << "]" << endl;
//    cout << "the1: [" << _thetaXX << "\t, " << _thetaYY << "]" << endl;
//    cout << "the2: [" << _thetaX << "\t, " << _thetaY << "]" << endl;

//    cout << "CurX: [" << cX[3] << "\t, " << cX[4] << "\t, " << cX[5] << "\t, " << cX[6] << "]" << endl;
//    cout << "DesX: [" << dX[0] << "\t, " << dX[1] << "\t, " << dX[2] << "]" << endl;
//    cout << "DesX: [" << dX[3] << "\t, " << dX[4] << "\t, " << dX[5] << "\t, " << dX[6] << "]" << endl;
    cout << "CurQ: [" << curQ[0] << "\t, " << curQ[1] << "\t, " << curQ[2] << "\t, " << curQ[3] << "\t, " << curQ[4] << "\t, " << curQ[5] << "\t, " << curQ[6] << "]" << endl;
    cout << "CurX: [" << curX[0] << "\t, " << curX[1] << "\t, " << curX[2] << "\t, " << curX[3] << "\t, " << curX[4] << "\t, " << curX[5] << "\t, " << curX[6] << "]" << endl;
    cout << "DesX: [" << desX[0] << "\t, " << desX[1] << "\t, " << desX[2] << "\t, " << desX[3] << "\t, " << desX[4] << "\t, " << desX[5] << "\t, " << desX[6] << "]" << endl;
    cout << "DesQ: [" << desQ[0] << "\t, " << desQ[1] << "\t, " << desQ[2] << "\t, " << desQ[3] << "\t, " << desQ[4] << "\t, " << desQ[5] << "\t, " << desQ[6] << "]" << endl;
//    cout << "befQ: [" << beforeQ[0] << "\t, " << beforeQ[1] << "\t, " << beforeQ[2] << "\t, " << beforeQ[3] << "\t, " << beforeQ[4] << "\t, " << beforeQ[5] << "\t, " << beforeQ[6] << "]" << endl;

}

/************************************************************************/
void ThreadImpl::saveInFileCsv()
{

    fprintf(fp,"\n%.4f", _FTLeftHand._initF.fx);
    fprintf(fp,",%.4f", _FTLeftHand._initF.fy);
    fprintf(fp,",%.4f", _FTLeftHand._initF.fz);
    fprintf(fp,",%.4f", _FTLeftHand._initT.mx);
    fprintf(fp,",%.4f", _FTLeftHand._initT.my);
    fprintf(fp,",%.4f", _FTLeftHand._initT.mz);
    fprintf(fp,",%.4f", _tray._zmp.x_zmp);
    fprintf(fp,",%.4f", _tray._zmp.y_zmp);

    fprintf(fp,",%.4f", curX[3]);
    fprintf(fp,",%.4f", curX[4]);
    fprintf(fp,",%.4f", curX[5]);
    fprintf(fp,",%.4f", curX[6]);

    fprintf(fp,",%.4f", _thetaXX);
    fprintf(fp,",%.4f", _thetaYY);
    fprintf(fp,",%.4f", _thetaX);
    fprintf(fp,",%.4f", _thetaY);

    fprintf(fp,",%.4f", act_time);

}

/************************************************************************/
void ThreadImpl::getCurrentTime()
{
    act_time = Time::now() - init_time;
    act_loop = Time::now() - init_loop;
}

/************************************************************************/

}   // namespace roboticslab
