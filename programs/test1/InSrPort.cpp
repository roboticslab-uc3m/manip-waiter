// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InSrPort.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <iomanip>

#include <fstream>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

namespace roboticslab
{

/************************************************************************/
void InSrPort::onRead(Bottle& FTsensor) {

    if (b!=250)    {    // deleting sensor offset
        offSetJR3(FTsensor);
        //iPositionControl->setPositionMode();
    }
    if (a==0 && b==250)    {    // moving the arm to the stability pose
        preprogrammedInitTrajectory();
        //iPositionControl->setPositionMode();
        a=1;    }

    if (b==250 && a==1 && inputAngle<12)   {
        getInitialTime();
        ReadFTSensor(FTsensor);
        AxesTransform1();
        AxesTransform2();
        ZMPcomp();
        LIPM3d();
        saveToFile();
        getCurrentTime();
        _diff_time = curr_time - init_loop;
//        printf("LoopTime = %f\n",_diff_time);

    }

    /*
     * if (umbral>rzmp)
     *      LIPM3d(_tray.xzmp, _tray.yzmp);
     *      pepinito = 0;
     * else                     // aun por desarrollar
     *      pepinito ++;
     *      if (pepinito>25)
     *          poseRefReturn();
     *
    */

}

/************************************************************************/
void InSrPort::preprogrammedInitTrajectory()
{
    fp = fopen("../data_zmp_bottle.csv","w+");

    leftArmIEncoders->getAxes(&numRobotJoints);
    CD_INFO("numRobotJoints: %d.\n",numRobotJoints);


/** ----- generate initial movement --------------- **/
    // -- 0.303928 0.347326 0.248109 0.008763 -0.999962 -0.000286 10.03554
    desireX[0] = 0.303928; // new X position
    desireX[1] = 0.347326; // new Y position
    desireX[2] = 0.248109; // new Z position
    desireX[3]= 1;
    desireX[4]= 0;
    desireX[5]= 0;
    desireX[6]= -10;

    if ( ! iCartesianSolver->invKin(desireX,currentQ,desireQ) )    {
        CD_ERROR("invKin failed.\n");    }

    if( ! leftArmIPositionControl2->positionMove( desireQ.data() )) {
        CD_WARNING("setPositions failed, not updating control this iteration.\n");    }

    yarp::os::Time::delay(3);  // provisional !!!!!!!!!!!!!!!!!!!!!!!!!!!!

    /*bool done = false;
    while( ! done )    {
        yarp::os::Time::delay(0.5);
        iPositionControl->checkMotionDone(&done);
        printf(".");
        fflush(stdout);
    }*/
    printf("end MOVE TO START POSITION\n");

/** ---- designate initial position --------------- **/ //comprobar funcionalidad
    if ( ! leftArmIEncoders->getEncoders( beforeQ.data() ) )    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;    }
    /** --------------------------------------------------- **/


    double initspe[7] = {3.0,3.0,3.0,3.0,3.0,3.0,0.0}; // --set NEW ref speed
    leftArmIPositionControl2->setRefSpeeds(initspe);
    double initacc[7] = {3.0,3.0,3.0,3.0,3.0,3,0.0}; // --set NEW ref accelaration
    leftArmIPositionControl2->setRefAccelerations(initacc);


fprintf(fp,"fx,fy,fz,Mx,My,Mz,Xzmp,Yzmp,CX3,CX4,CX5,CX6,TX1,TY1,TX2,TY2,initT,currT,diffT");
    yarp::os::Time::delay(10);
    return;
}

/************************************************************************/
void InSrPort::ReadFTSensor(Bottle& FTsensor){
    /**     * Reading input messages from JR3 SENSOR    **/

    _jr3._initF.fx = FTsensor.get(0).asDouble();
    _jr3._initF.fy = FTsensor.get(1).asDouble();
    _jr3._initF.fz = FTsensor.get(2).asDouble();
    _jr3._initT.mx = FTsensor.get(3).asDouble();
    _jr3._initT.my = FTsensor.get(4).asDouble();
    _jr3._initT.mz = FTsensor.get(5).asDouble();

}

/************************************************************************/
void InSrPort::AxesTransform1(){
/**     * Transformation matrix between TEO_body_axes (world) and Jr3_axes
     * with horizontal tray (waiter)    **/

    _tray._F.fx = + _jr3._initF.fz - _off._F.fz;
    _tray._F.fy = + _jr3._initF.fy - _off._F.fy;
    _tray._F.fz = + _jr3._initF.fx + _off._F.fx;
    _tray._M.mx = + _jr3._initT.mz - _off._M.mz;
    _tray._M.my = + _jr3._initT.my - _off._M.my;
    _tray._M.mz = + _jr3._initT.mx - _off._M.mx;

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
void InSrPort::AxesTransform2(){
   //     Force/torque Transformation depending on the TCP orientation.

    if ( ! leftArmIEncoders->getEncoders( currentQ.data() ) )    { //obtencion de los valores articulares (encoders absolutos)
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;    }
    if (fabs(currentQ[0] - beforeQ[0]) < 0.2)  {
        currentQ[0] = beforeQ[0];    }
    if (fabs(currentQ[1] - beforeQ[1])<0.2)  {
        currentQ[1] = beforeQ[1];    }
    if (fabs(currentQ[2] - beforeQ[2])<0.2)  {
        currentQ[2] = beforeQ[2];    }
    if (fabs(currentQ[3] - beforeQ[3])<0.5)  {
        currentQ[3] = beforeQ[3];    }
    if (fabs(currentQ[4] - beforeQ[4])<0.5)  {
        currentQ[4] = beforeQ[4];    }
    if (fabs(currentQ[5] - beforeQ[5])<0.5)  {
        currentQ[5] = beforeQ[5];    }
    if (fabs(currentQ[6] - beforeQ[6])<0.5)  {
        currentQ[6] = beforeQ[6];    }


    if ( ! iCartesianSolver->fwdKin(currentQ,currentX) )    {
        CD_ERROR("fwdKin failed.\n");    }

    //normalizando (no hace falta xq ya esta predefinido)
    //_normVector = sqrt(pow((currentX[3]),2) + pow((currentX[4]),2) + pow((currentX[5]),2));
    //currentX[3] = currentX[3] / _normVector;
    //currentX[4] = currentX[4] / _normVector;
    //currentX[5] = currentX[5] / _normVector;

    //redondeando los quaternios
    //int i;
    //for (i=3;i<=6;i++)   {
        //if (currentX[i]>0)  {
        //    currentX[i] = round (currentX[i]); //} // ceil or floor
        //else{    currentX[i] = ceil (currentX[i]);    }    }

    //convirtiendo de grados a radianes
    //angle= currentX[6];

    currentX[6]=(currentX[6]/180)*3.1415926;
    if ((currentX[6]<0.015))     {
        currentX[3] = 0;
        currentX[4] = 0;
        currentX[5] = 0;    }
    //convirtiendo en quaternios
    quat[0] = cos(currentX[6]/2); // angulo theta
    quat[1] = currentX[3] * sin(currentX[6]/2); // eje X
    quat[2] = currentX[4] * sin(currentX[6]/2); // eje Y
    quat[3] = currentX[5] * sin(currentX[6]/2); // eje Z, siempre a cero

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
void InSrPort::ZMPcomp()    {
    /**     * Bottle ZMP measurements    **/
/*
    if ( ! iEncoders->getEncoders( currentQ.data() ) )    { //obtencion de los valores articulares (encoders absolutos)
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        std::cout << "ZMP: [dentro]" << endl;
        return;    }
*/
    if ( ! iCartesianSolver->fwdKin(currentQ,currentX) )    {
        CD_ERROR("fwdKin failed.\n");    }

    //angle=(currentX[6]/180)*3.1415926; // transformando a rad
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
void InSrPort::LIPM3d()
{

    // ref[x,y,z]: 0.302982 0.34692 0.247723
    desireX[0] = 0.302982; // new X position
    desireX[1] = 0.34692; // new Y position
    desireX[2] = 0.247723; // new Z position

    if (w<200)  {
        w++;
        printf("\n %d",w);
        return;
    }
    else    {
        w=0;
        desireX[3]= 1;
        desireX[4]= 0;
        desireX[5]= 0;
        desireX[6]= inputAngle;
        inputAngle++;
    }

    if ( ! iCartesianSolver->invKin(desireX,currentQ,desireQ) )    {
        CD_ERROR("invKin failed.\n");    }

    if( ! leftArmIPositionControl2->positionMove( desireQ.data() )) {
        CD_WARNING("setPositions failed, not updating control this iteration.\n");    }

//    yarp::os::Time::delay(3);
    return;

}

/************************************************************************/
void InSrPort::saveToFile(){
//    _modFS = sqrt(pow((_tray._F.fx),2) + pow((_tray._F.fy),2) + pow((_tray._F.fz),2));
//    _modFF = sqrt(pow((FF[1]),2) + pow((FF[2]),2) + pow((FF[3]),2));

//    cout << "CurX: [" << currentX[3] << "\t, " << currentX[4] << "\t, " << currentX[5] << "\t, " << currentX[6] << "]" << endl;
//    cout << "Quat: [" << quatC[0] << "\t, " << quatC[1] << "\t, " << quatC[2] << "\t, " << quatC[3] << "]" << endl;
//    cout << "F_init: [" << _tray._F.fx << "\t, " << _tray._F.fy << "\t, " << _tray._F.fz << "\t " << "]" << endl;//<< FF[1] << "]" << endl;
//    cout << "F_finl: [" << FF[1] << "\t, " << FF[2] << "\t, " << FF[3] << "\t " << "]" << endl;//<< FF[1] << "]" << endl;
//    cout << "torque: [" << _tray._M.mx << "\t, " << _tray._M.my << "\t, " << _tray._M.mz << "\t " << "]" << endl;//<< FF[1] << "]" << endl;

//    cout << "F_X: [" << _tray._F.fx << "\t, " << FF[1] << "]" << endl;//<< FF[1] << "]" << endl;
//    cout << "F_Y: [" << _tray._F.fy << "\t, " << FF[2] << "]" << endl;//<< FF[2] << "]" << endl;
//    cout << "F_Z: [" << _tray._F.fz << "\t, " << FF[3] << "]" << endl;//<< FF[3] << "]" << endl;
    cout << "ZMP: [" << _tray._zmp.x_zmp << "\t, " << _tray._zmp.y_zmp << "]" << endl;
//    cout << "mod: [" << _modFS << "\t, " << _modFF << "]" << endl;
//    cout << "the1: [" << _thetaXX << "\t, " << _thetaYY << "]" << endl;
//    cout << "the2: [" << _thetaX << "\t, " << _thetaY << "]" << endl;

//    cout << "CurX: [" << currentX[3] << "\t, " << currentX[4] << "\t, " << currentX[5] << "\t, " << currentX[6] << "]" << endl;
//    cout << "DesX: [" << desireX[3] << "\t, " << desireX[4] << "\t, " << desireX[5] << "\t, " << desireX[6] << "]" << endl;
//    cout << "CurX: [" << currentX[0] << "\t, " << currentX[1] << "\t, " << currentX[2] << "\t, " << currentX[3] << "\t, " << currentX[4] << "\t, " << currentX[5] << "\t, " << currentX[6] << "]" << endl;
//    cout << "DesX: [" << desireX[0] << "\t, " << desireX[1] << "\t, " << desireX[2] << "\t, " << desireX[3] << "\t, " << desireX[4] << "\t, " << desireX[5] << "\t, " << desireX[6] << "]" << endl;
//    cout << "befQ: [" << beforeQ[0] << "\t, " << beforeQ[1] << "\t, " << beforeQ[2] << "\t, " << beforeQ[3] << "\t, " << beforeQ[4] << "\t, " << beforeQ[5] << "\t, " << beforeQ[6] << "]" << endl;
//    cout << "curQ: [" << currentQ[0] << "\t, " << currentQ[1] << "\t, " << currentQ[2] << "\t, " << currentQ[3] << "\t, " << currentQ[4] << "\t, " << currentQ[5] << "\t, " << currentQ[6] << "]" << endl;

    /*CD_DEBUG_NO_HEADER("F_init:[");
    CD_DEBUG_NO_HEADER("%f \t",_tray._F.fx);
    CD_DEBUG_NO_HEADER("%f \t",_tray._F.fy);
    CD_DEBUG_NO_HEADER("%f \t",_tray._F.fz);
    CD_DEBUG_NO_HEADER("] - F_final:[ ");
    CD_DEBUG_NO_HEADER("%f \t",FF[1]);
    CD_DEBUG_NO_HEADER("%f \t",FF[2]);
    CD_DEBUG_NO_HEADER("%f \t",FF[3]);
    CD_DEBUG_NO_HEADER("]\n ");
    CD_DEBUG_NO_HEADER("%f \t",_modFS);
    CD_DEBUG_NO_HEADER("%f \t",_modFF);
    CD_DEBUG_NO_HEADER("\n ");*/

    /*ofstream out;
    if(iteration==1) {
        out.open("cabeza.txt",ios::trunc);
    }
    else {
        out.open("cabeza.txt",ios::app);
    }
    out <<  _tray._zmp.x_zmp << " " <<  _tray._zmp.y_zmp << " " << endl;
    out.close();
    iteration ++;
*/
//    fprintf(fp,"fx,fy,fz,Mx,My,Mz,Xzmp,Yzmp,CX3,CX4,CX5,CX6,TX1,TY1,TX2,TY2,initT,currT,diffT");

    fprintf(fp,"\n%.2f", _jr3._initF.fx);
    fprintf(fp,",%.2f", _jr3._initF.fy);
    fprintf(fp,",%.2f", _jr3._initF.fz);
    fprintf(fp,",%.2f", _jr3._initT.mx);
    fprintf(fp,",%.2f", _jr3._initT.my);
    fprintf(fp,",%.2f", _jr3._initT.mz);
    fprintf(fp,",%.2f", _tray._zmp.x_zmp);
    fprintf(fp,",%.2f", _tray._zmp.y_zmp);
//    fprintf(fp,",%.2f", _rzmp);

    fprintf(fp,",%.2f", currentX[3]);
    fprintf(fp,",%.2f", currentX[4]);
    fprintf(fp,",%.2f", currentX[5]);
    fprintf(fp,",%.2f", currentX[6]);

//    fprintf(fp,",%.2f", desireX[3]);
//    fprintf(fp,",%.2f", desireX[4]);
//    fprintf(fp,",%.2f", desireX[5]);
//    fprintf(fp,",%.2f", desireX[6]);

    fprintf(fp,",%.2f", _thetaXX);
    fprintf(fp,",%.2f", _thetaYY);
    fprintf(fp,",%.2f", _thetaX);
    fprintf(fp,",%.2f", _thetaY);

    fprintf(fp,",%.2f", init_loop);
    fprintf(fp,",%.2f", curr_time);
    fprintf(fp,",%.2f", _diff_time);

/*    fprintf(fp,",%.15f", _eval_x._zmp_error);
    fprintf(fp,",%.15f", _eval_x._zmp_ref);
    fprintf(fp,",%10f", _eval_x._u);
    fprintf(fp,",%10f", _tray.xzmp);
    fprintf(fp,",%10f", _tray.yzmp);
    fprintf(fp,",%f", angle_x);
    fprintf(fp,",%f", vel);*/

    //        fprintf(fp,",%.15f", _eval_y._r);
    //        fprintf(fp,",%.15f", _yzmp);
    //        fprintf(fp,",%.15f", _eval_y.y);
    //        fprintf(fp,",%f", angle_y);
}

/************************************************************************/
void InSrPort::offSetJR3(Bottle& FTsensor){

    /**     * Offset JR3 measurements    **/

    ReadFTSensor(FTsensor);
    _off._F.fx += -9.72 - _jr3._initF.fx; //9.72N = 0.99kg * 9.81mm/s2
    _off._F.fy += + _jr3._initF.fy;
    _off._F.fz += + _jr3._initF.fz;
    _off._M.mx += + _jr3._initT.mx;
//    _off._M.my += 0.243 - _jr3._initT.my; //0.243Nm = 9.72N * 0.025m
    _off._M.my += + _jr3._initT.my; //testeando *************
    _off._M.mz += + _jr3._initT.mz;

    b++;
    printf("........................%d\n", b);

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
void InSrPort::mediumJR3(Bottle& FTsensor){

    /**     * Offset JR3 measurements    **/

    ReadFTSensor(FTsensor);
    _med._initF.fx += _jr3._initF.fx;
    _med._initF.fy += _jr3._initF.fy;
    _med._initF.fz += _jr3._initF.fz;
    _med._initT.mx += _jr3._initT.mx;
    _med._initT.my += _jr3._initT.my;
    _med._initT.mz += _jr3._initT.mz;

    pepinito++;

    if (pepinito==10) {
        _jr3._initF.fx = _med._initF.fx / pepinito;
        _jr3._initF.fy = _med._initF.fy / pepinito;
        _jr3._initF.fz = _med._initF.fz / pepinito;
        _jr3._initT.mx = _med._initT.mx / pepinito;
        _jr3._initT.my = _med._initT.my / pepinito;
        _jr3._initT.mz = _med._initT.mz / pepinito;
    }

}

/************************************************************************/
void InSrPort::getInitialTime(){
    init_loop = yarp::os::Time::now();
}

/************************************************************************/
void InSrPort::getCurrentTime(){
    curr_time = yarp::os::Time::now();
}

/************************************************************************/

}   // namespace roboticslab
