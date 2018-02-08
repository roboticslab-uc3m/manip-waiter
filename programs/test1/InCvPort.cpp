// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InCvPort.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include <fstream>

//static FILE *fp;

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

namespace roboticslab
{

/************************************************************************/
void InCvPort::setFollow(int value)
{
    follow = value;
}

/************************************************************************/
void InCvPort::onRead(Bottle& FTsensor) {


    if (a==0)    {
        preprogrammedInitTrajectory();
        //iPositionControl->setPositionMode();
        a=1;
    }

    //poseRefCalculate();      // aun por desarrollar

    ReadFTSensor(FTsensor);
    AxesTransform();
    ZMPcomp();
    LIPM3d();
    saveToFile();

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
bool InCvPort::preprogrammedInitTrajectory()
{
    iEncoders->getAxes(&numRobotJoints);
    CD_INFO("numRobotJoints: %d.\n",numRobotJoints);


/** ----- generate initial movement --------------- **/
    //iPositionControl->setPositionMode();
    printf("begin MOVE TO START POSITION\n");
    double initpos[7] = {-30,0,0,-90,0,30,0};
    //iPositionControl->positionMove(initpos);
    //posicionamiento temporal hasta arreglar set poss
    iPositionControl->positionMove(0,-30);
    iPositionControl->positionMove(1,0);
    iPositionControl->positionMove(2,0);
    iPositionControl->positionMove(3,-90);
    iPositionControl->positionMove(4,0);
    iPositionControl->positionMove(5,30);

    yarp::os::Time::delay(10);  // provisional !!!!!!!!!!!!!!!!!!!!!!!!!!!!
    /*bool done = false;
    while( ! done )    {
        yarp::os::Time::delay(0.5);
        iPositionControl->checkMotionDone(&done);
        printf(".");
        fflush(stdout);
    }*/
    printf("end MOVE TO START POSITION\n");

/** ---- designate initial position --------------- **/ //comprobar funcionalidad
    std::vector<double> beforeQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( beforeQ.data() ) )    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return false;
    }
    /** --------------------------------------------------- **/


/** ----- set NEW ref speed --------------- **/
    double initspe[7] = {100.0,100.0,100.0,100.0,100.0,100.0,0.0};
    iPositionControl->setRefSpeeds(initspe);
    /** --------------------------------------------------- **/


/** ----- set NEW ref accelaration --------------- **/
    double initacc[7] = {50.0,50.0,50.0,50.0,50.0,50,0.0};
    iPositionControl->setRefAccelerations(initacc);
    /** --------------------------------------------------- **/


    yarp::os::Time::delay(3);
    return true;
}

/************************************************************************/
void InCvPort::ReadFTSensor(Bottle& FTsensor){
    /**
     * Reading input messages from JR3 SENSOR
    **/

    _sensor3.fx = FTsensor.get(0).asDouble();
    _sensor3.fy = FTsensor.get(1).asDouble();
    _sensor3.fz = FTsensor.get(2).asDouble();
    _sensor3.mx = FTsensor.get(3).asDouble();
    _sensor3.my = FTsensor.get(4).asDouble();
    _sensor3.mz = FTsensor.get(5).asDouble();
}

/************************************************************************/
void InCvPort::AxesTransform(){
    /**
     * Transformation matrix between TEO_body_axes (world) and Jr3_axes
     * with horizontal tray (waiter)
    **/

    _tray.fx = + _sensor3.fz;
    _tray.fy = - _sensor3.fy;
    _tray.fz = + _sensor3.fx;
    _tray.mx = + _sensor3.mz;
    _tray.my = - _sensor3.my;
    _tray.mz = + _sensor3.mx;
}

/************************************************************************/
void InCvPort::ZMPcomp(){
    /**
     * Bottle ZMP measurements
    **/

    if (_tray.fz==0)    {
        _tray.xzmp = _tray.xzmp; // Milimetros
        _tray.yzmp = _tray.yzmp; // Milimetros
    }else{
        _tray.xzmp = -((- (_tray.my) / (_tray.fz)) + _d); // Metros
        _tray.yzmp = ((_tray.mx) / _tray.fz); // Metros
    }

    if (_tray.xzmp>0.075)    { //limitando el maximo ZMP en X positivo
        _tray.xzmp = 0.075;
    }
    if (_tray.xzmp<-0.075)    {//limitando el maximo ZMP en X negativo
        _tray.xzmp = -0.075;
    }

    if (_tray.yzmp>0.075)    {//limitando el maximo ZMP en Y positivo
        _tray.yzmp = 0.075;
    }
    if (_tray.yzmp<-0.075)    {//limitando el maximo ZMP en Y negativo
        _tray.yzmp = -0.075;
    }

    _rzmp = sqrt(pow(_tray.xzmp,2) + pow(_tray.yzmp,2));

    //cout << "ZMP: [" << _tray.xzmp << ", " << _tray.yzmp << "]" << endl;
}

/************************************************************************/
void InCvPort::LIPM3d(){
    //Generacion de la actuacion a los motores (CONTROL)


    std::vector<double> currentQ(numRobotJoints), desireQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( currentQ.data() ) )    { //obtencion de los valores articulares (encoders absolutos)
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;
    }

    std::vector<double> currentX, desireX;
    if ( ! iCartesianSolver->fwdKin(currentQ,currentX) )    {
        CD_ERROR("fwdKin failed.\n");
    }

    desireX = currentX; // 6 = 6
    _rWorkSpace = sqrt(pow((currentX[0]-0.5),2) + pow((currentX[1]-0.375),2));

/**
    if ( (angle >= 70) && (angle < 85) )    {   //Correction 01. Move arm Y right.
        if( currentX[1] > 0.30 )        {
            printf("THE BOTTLE GOES RIGHT \n");
            desireX[1] = currentX[1] - (0.2*cos(angle));
            pepinito = 1;
        }
        else if( currentX[1] <= 0.30 )        {
            printf("¡¡¡ BOTTLE FALL RIGHT !!! \n");
            return;

            //desireX[1] = 0.346914;
            //desireX[1] = currentX[1];
            //pepinito = 3;


            //if( ! iPositionControl->positionMove( beforeQ.data() ))            {
            //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
            //
            // beforeQ = currentQ; ???? por confirmar
            //return;
        }
    }
    else if( (angle > 95) && (angle <= 110) )    {   //Correction 02. Move arm Y left.
        if( currentX[1] < 0.45 )        {
             printf("THE BOTTLE GOES LEFT \n");
             desireX[1] = currentX[1] - (0.2*cos(angle));
             pepinito = 1;
        }
        else if( currentX[1] >= 0.45 )                {
             printf("¡¡¡ BOTTLE FALL LEFT !!! \n");
             return;

             //desireX[1] = 0.346914;
             //desireX[1] = currentX[1];
             //pepinito = 3;

             //if( ! iPositionControl->positionMove( beforeQ.data() ))                    {
             //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
             //}
             // beforeQ = currentQ; ???? por confirmar
             //return;
        }
    }
    else    {      //if(z>=88 && z<=92)
        printf("THE BOTTLE IS IN EQUILIBRIUM \n");
        pepinito = 3;
        //return;

        //desireX[1] = 0.346914;
        //pepinito = 3;

        //if( ! iPositionControl->positionMove( beforeQ.data() ))        {
        //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
        //}
        // beforeQ = currentQ; ???? por confirmar
        //return;
    }
**/

    if ((_rzmp>0.020) && (_rWorkSpace<0.075))   { // será necesario programar los limites en los ejes X e Y.
        desireX[0] = currentX[0] + _tray.xzmp; // new X position
        desireX[1] = currentX[1] + _tray.yzmp; // new Y position
    }
    if ((_rzmp>0.020) && (_rWorkSpace>0.075))   { // será necesario programar los limites en los ejes X e Y.
        printf("¡¡¡ BOTTLE FALL !!! \n");
        //return;
    }
    if ((_rzmp<0.020))   { // será necesario programar los limites en los ejes X e Y.
        desireX[0] = 0.526565; // new X position
        desireX[1] = 0.346227; // new Y position
        printf("THE BOTTLE IS IN EQUILIBRIUM \n");
    }

    desireX[2] = 0.309705; // Z position

    // ref: 0.526565 0.346227 0.309705 -0.91063 0.409172 -0.057711 0.610729
    desireX[3] = -0.351340;    //
    desireX[4] = 0.707474;     //
    desireX[5] = 0.613221;     //  Orientation
    desireX[6] = 0.351825;    //

    if ( ! iCartesianSolver->invKin(desireX,currentQ,desireQ) )    {
        CD_ERROR("invKin failed.\n");
    }

    CD_DEBUG_NO_HEADER("[currentX]");
    for(int i=0;i<7;i++)
        CD_DEBUG_NO_HEADER("%f ",currentX[i]);
    CD_DEBUG_NO_HEADER("\n ");

    CD_DEBUG_NO_HEADER("[desireX]");
    for(int i=0;i<7;i++)
        CD_DEBUG_NO_HEADER("%f ",desireX[i]);
    CD_DEBUG_NO_HEADER("\n ");

    CD_DEBUG_NO_HEADER("[ZMP values]");
    CD_DEBUG_NO_HEADER("%f ",_tray.xzmp);
    CD_DEBUG_NO_HEADER("%f ",_tray.yzmp);
    CD_DEBUG_NO_HEADER("%f ",_rWorkSpace);
    CD_DEBUG_NO_HEADER("\n ");


    if( ! iPositionControl->positionMove( desireQ.data() ))        {
        CD_WARNING("setPositions failed, not updating control this iteration.\n");
    }

    return;

}

/************************************************************************/
void InCvPort::saveToFile()
{

    ofstream out;
    if(iteration==1) {
        out.open("cabeza.txt",ios::trunc);
    }
    else {
        out.open("cabeza.txt",ios::app);
    }
    out <<  _tray.xzmp << " " <<  _tray.yzmp << " " << endl;
    out.close();
    iteration ++;

//    fprintf(fp,"\n%d", n);
//    fprintf(fp,",%.4f", _dt);
//    fprintf(fp,",%.15f", X);
//    fprintf(fp,",%.15f", _eval_x.y);
//    fprintf(fp,",%.15f", _eval_x._zmp_error);
//    fprintf(fp,",%.15f", _eval_x._zmp_ref);
//    fprintf(fp,",%10f", _eval_x._u);
//    fprintf(fp,",%10f", _tray.xzmp);
//    fprintf(fp,",%10f", _tray.yzmp);
//    fprintf(fp,",%f", angle_x);
//    fprintf(fp,",%f", vel);

    //        fprintf(fp,",%.15f", _eval_y._r);
    //        fprintf(fp,",%.15f", _yzmp);
    //        fprintf(fp,",%.15f", _eval_y.y);
    //        fprintf(fp,",%f", angle_y);
}

/************************************************************************/
}  // namespace roboticslab

