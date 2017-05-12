// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InSrPort.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include <fstream>

//static FILE *fp;

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

namespace teo
{

/************************************************************************/
void InSrPort::setFollow(int value)
{
    follow = value;
}

/************************************************************************/
void InSrPort::onRead(Bottle& FTsensor) {


    if (a==0)    {
        preprogrammedInitTrajectory();
        iPositionControl->setPositionMode();
        a=1;
    }

    //poseRefCalculate();      // aun por desarrollar

    ReadFTSensor(FTsensor);
    AxesTransform1();
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
void InSrPort::strategyPositionDirect(Bottle& FTsensor)
{

    if (a==0)    {
        preprogrammedInitTrajectory();
        //iPositionDirect->setPositionDirectMode();
        iPositionControl->setPositionMode();
        a=1;
    }

/** -------------------READING INPUT MESSAGES FROM VISION SENSOR-------------------**/
    //double x = b.get(0).asDouble(); //Data pxXpos
    //double y = b.get(1).asDouble(); //Data pxYpos
    double angle = FTsensor.get(2).asDouble(); //Angle
    /** --------------------------------------------------- **/


    // (0.526938 0.346914 0.312769 -1.0 0.00 -0.00 90.00) -- posicion inicial en cartesiano--

/** ----- Obtain current joint position --------------- **/
    std::vector<double> currentQ(numRobotJoints);
    std::vector<double> desireQ(numRobotJoints);

    if ( ! iEncoders->getEncoders( currentQ.data() ) )    { //obtencion de los valores articulares (encoders absolutos)
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;
    }
    /** --------------------------------------------------- **/


/** ----- Obtain current cartesian position ---------- **/
    std::vector<double> currentX, desireX;
    if ( ! iCartesianSolver->fwdKin(currentQ,currentX) )    {
        CD_ERROR("fwdKin failed.\n");
    }
/*    CD_DEBUG_NO_HEADER("[currentX]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",currentX[i]);
    CD_DEBUG_NO_HEADER("\n ");*/
    /** --------------------------------------------------- **/


/** -----------------------CONTROL------------------------ **/

    desireX = currentX; // 6 = 6

    if ( (angle >= 70) && (angle < 85) )    {   //Correction 01. Move arm Y right.
        if( currentX[1] > 0.30 )        {
            printf("THE BOTTLE GOES RIGHT \n");
            desireX[1] = currentX[1] - (0.2*cos(angle));
            pepinito = 1;
        }
        else if( currentX[1] <= 0.30 )        {
            printf("¡¡¡ BOTTLE FALL RIGHT !!! \n");
            return;
/*
            //desireX[1] = 0.346914;
            desireX[1] = currentX[1];
            pepinito = 3;
*/

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

/*
             //desireX[1] = 0.346914;
             desireX[1] = currentX[1];
             pepinito = 3;
*/

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
/*
        desireX[1] = 0.346914;
        pepinito = 3;
*/
        //if( ! iPositionControl->positionMove( beforeQ.data() ))        {
        //    CD_WARNING("setPositions failed, not updating control this iteration.\n");
        //}
        // beforeQ = currentQ; ???? por confirmar
        //return;
    }
    desireX[0] = 0.526938;
    desireX[2] = 0.312769;
    desireX[3] = -1;
    desireX[4] = 0;
    desireX[5] = 0;
    desireX[6] = 90;

    if ( ! iCartesianSolver->invKin(desireX,currentQ,desireQ) )    {
        CD_ERROR("invKin failed.\n");
    }

    CD_DEBUG_NO_HEADER("[desireX]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",desireX[i]);
    CD_DEBUG_NO_HEADER("\n ");

    //--------------------------------------------------------------
/*
    CD_DEBUG_NO_HEADER("[beforeQ]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",currentQ[i]);
    CD_DEBUG_NO_HEADER("\n ");
    CD_DEBUG_NO_HEADER("[currentQ]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",currentQ[i]);
    CD_DEBUG_NO_HEADER("\n ");
    CD_DEBUG_NO_HEADER("[desireQ]");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",desireQ[i]);
    CD_DEBUG_NO_HEADER("\n");
*/
    double initpos[7] = {-30,0,0,-90,0,30,0};
    switch (pepinito)    {
    case 1:
        if( ! iPositionControl->positionMove( desireQ.data() ))        {
            CD_WARNING("setPositions failed, not updating control this iteration.\n");
        }
        break;
    case 2: // no se usa
        if( ! iPositionControl->positionMove( currentQ.data() ))        {
            CD_WARNING("setPositions failed, not updating control this iteration.\n");
        }
        break;
    case 3:
        /*currentQ = std::vector<double>(initpos);
        for(int i=0;i<numRobotJoints;i++)
            currentQ.data(i) = initpos[i];
        CD_DEBUG_NO_HEADER("[currentQ]");
        for(int i=0;i<numRobotJoints;i++)
            CD_DEBUG_NO_HEADER("%f ",currentQ[i]);
        CD_DEBUG_NO_HEADER("\n ");
        double initpos[7] = {-30,0,0,-90,0,30,0};
        */

        /*
        iPositionControl->positionMove(0, -30);
        iPositionControl->positionMove(1, 0);
        iPositionControl->positionMove(2, 0);
        iPositionControl->positionMove(3, -90);
        iPositionControl->positionMove(4, 0);
        iPositionControl->positionMove(5, 30);
        iPositionControl->positionMove(6, 0);
        */

        if( ! iPositionControl->positionMove( initpos ))        {
            CD_WARNING("setPositions failed, not updating control this iteration.\n");
        }

        /*bool done = false;     // para comprobar si se hace el movimiento correctamente
        while( ! done )    {
            yarp::os::Time::delay(0.5);
            iPositionControl->checkMotionDone(&done);
            printf(".");
            fflush(stdout);
        }*/

        break;
    default:
        break;
    }

    //beforeQ = currentQ;

    return;

}

/************************************************************************/
void InSrPort::strategyVelocity(Bottle& FTsensor)
{

    if (a==0)    {
        preprogrammedInitTrajectory();
        iVelocityControl->setVelocityMode();
        a=1;
    }

//-------------------READING INPUT MESSAGES FROM VISION SENSOR--------------------

    //double x = b.get(0).asDouble(); //Data pxXpos
    //double y = b.get(1).asDouble(); //Data pxYpos
    double angle = FTsensor.get(2).asDouble(); //Angle

//------------------------CONTROL------------------------

    //-- Obtain current joint position
    std::vector<double> currentQ(numRobotJoints);
    if ( ! iEncoders->getEncoders( currentQ.data() ) )    {
        CD_WARNING("getEncoders failed, not updating control this iteration.\n");
        return;
    }

    std::vector<double> currentX, xdotd(6, 0.0); // derivada de la posicion cartesiana deseada

    //-- Perform forward kinematics to obtain cartesian position
    if ( ! iCartesianSolver->fwdKin(currentQ,currentX) )    {
        CD_ERROR("fwdKin failed.\n");
        return;
    }

    //-- 0.526938 0.346914 0.312769 -1.0 0.00 -0.00 90.00

/*    if ( x[0] > 0.526938+0.001 )
        xdotd[0] = -0.01;
    if ( x[0] < 0.526938-0.001 )
        xdotd[0] = 0.01;
    if ( x[2] > 0.312769+0.001 )
        xdotd[2] = -0.01;
    if ( x[2] < 0.312769-0.001 )
        xdotd[2] = 0.01;
*/

    std::vector<double> desireX;
    if ( (angle >= 70) && (angle < 88) )  //Correction 01. Move arm Y right.
    {
        if( currentX[1] >= 0.25 )        {
            printf("THE BOTTLE GOES RIGHT \n");
            xdotd[1] = -0.05; // [1] corresponds to Y axis
            desireX[1] = currentX[1] - 0.02;
            pepinito = 1;
        }
        else if( currentX[1] < 0.25 )        {
            printf("¡¡¡ BOTTLE FALL RIGHT !!! \n");
            return;
            desireX[1] = 0.346914;
            pepinito = 3;
        }
    }
    else if( (angle > 92) && (angle <= 110) )  //Correction 02. Move arm Y right.
    {
        if( currentX[1] <= 0.45 )        {
             printf("THE BOTTLE GOES LEFT \n");
             desireX[1] = currentX[1] + 0.02;
             xdotd[1] = 0.05; // [1] corresponds to Y axis
             pepinito = 1;
        }
        else if( currentX[1] > 0.45 )                {
             printf("¡¡¡ BOTTLE FALL LEFT !!! \n");
             return;
             desireX[1] = 0.346914;
             pepinito = 3;
        }
    }
    else //if(z>=88 && z<=92)
    {
        printf("THE BOTTLE IS IN EQUILIBRIUM \n");
        return;
        desireX[1] = 0.346914;
        pepinito = 3;
    }

    //-- Compute joint velocity commands and send to robot.
    std::vector<double> commandQdot;
    if (! iCartesianSolver->diffInvKin(currentQ,xdotd,commandQdot) )    {
        CD_WARNING("diffInvKin failed, not updating control this iteration.\n");
        return;
    }

    for(int i=0;i<commandQdot.size();i++)    {
        if( fabs(commandQdot[i]) > DEFAULT_QDOT_LIMIT)        {
            CD_ERROR("diffInvKin too dangerous, STOP!!!.\n");
            for(int i=0;i<commandQdot.size();i++)
                commandQdot[i] = 0;
        }
    }

    CD_DEBUG_NO_HEADER("[MOVV] ");
    for(int i=0;i<6;i++)
        CD_DEBUG_NO_HEADER("%f ",xdotd[i]);
    CD_DEBUG_NO_HEADER("-> ");
    for(int i=0;i<numRobotJoints;i++)
        CD_DEBUG_NO_HEADER("%f ",commandQdot[i]);
    CD_DEBUG_NO_HEADER("[deg/s]\n");

    commandQdot[0] = 0.1;
    if( ! iVelocityControl->velocityMove( commandQdot.data() ) )    {
        CD_WARNING("velocityMove failed, not updating control this iteration.\n");
    }

    return;
}

/************************************************************************/
bool InSrPort::preprogrammedInitTrajectory()
{
    iEncoders->getAxes(&numRobotJoints);
    CD_INFO("numRobotJoints: %d.\n",numRobotJoints);


/** ----- generate initial movement --------------- **/
    iPositionControl->setPositionMode();
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
void InSrPort::ReadFTSensor(Bottle& FTsensor){
    /**
     * Reading input messages from JR3 SENSOR
    **/

    _jr3._initF.fx = FTsensor.get(0).asDouble();
    _jr3._initF.fy = FTsensor.get(1).asDouble();
    _jr3._initF.fz = FTsensor.get(2).asDouble();
    _jr3._initT.mx = FTsensor.get(3).asDouble();
    _jr3._initT.my = FTsensor.get(4).asDouble();
    _jr3._initT.mz = FTsensor.get(5).asDouble();
}

/************************************************************************/
void InSrPort::AxesTransform1(){
    /**
     * Force/torque Transformation depending on the TCP orientation.
    **/

    std::vector<double> currentX, desireX;
    if ( ! iCartesianSolver->fwdKin(currentQ,currentX) )    {
        CD_ERROR("fwdKin failed.\n");    }


    //normalizando
    _normVector = sqrt(pow((currentX[3]),2) + pow((currentX[4]),2));
    currentX[3] = currentX[3] / _normVector;
    currentX[4] = currentX[4] / _normVector;
    //convirtiendo en quaternios
    quat[0] = cos (currentX[6]/2);
    quat[1] = sen (currentX[3]/2);
    quat[2] = sen (currentX[4]/2);
    quat[3] = 0;
    //calculando quaternio conjugado
    quatC = quat;
    quatC[1] = - quat[1];
    quatC[2] = - quat[2];

    //transformada      preFinalForce   = quat              *   _initF
    //                  Final_F     = preFF    *   quatC
    preFF[0]=((-quat[1]*_jr3._initF.fx) - (quat[2]*_jr3._initF.fy) - (quat[3]*_jr3._initF.fz));
    preFF[1]=((quat[0]*_jr3._initF.fx) + (quat[2]*_jr3._initF.fz) - (quat[3]*_jr3._initF.fy));
    preFF[2]=((quat[0]*_jr3._initF.fy) - (quat[1]*_jr3._initF.fz) + (quat[3]*_jr3._initF.fx));
    preFF[3]=((quat[0]*_jr3._initF.fz) + (quat[1]*_jr3._initF.fy) - (quat[2]*_jr3._initF.fx));
    FF[0]=((preFF[0]*quatC[0]) - (preFF[1]*quatC[1]) - (preFF[2]*quatC[2])) - (preFF[3]*quatC[3]);
    FF[1]=((preFF[0]*quatC[1]) + (preFF[1]*quatC[0]) + (preFF[2]*quatC[3])) - (preFF[3]*quatC[2]); //fx
    FF[2]=((preFF[0]*quatC[2]) - (preFF[1]*quatC[3]) + (preFF[2]*quatC[0])) + (preFF[3]*quatC[1]); //fy
    FF[3]=((preFF[0]*quatC[3]) + (preFF[1]*quatC[2]) - (preFF[2]*quatC[1])) + (preFF[3]*quatC[0]); //fz

    //transformada      preFM   = quat              *   _initM
    //                  FM     = _preFM    *   quatC
    preFM[0]=((-quat[1]*_jr3._initT.mx) - (quat[2]*_jr3._initT.my) - (quat[3]*_jr3._initT.mz));
    preFM[1]=((quat[0]*_jr3._initT.mx) + (quat[2]*_jr3._initT.mz) - (quat[3]*_jr3._initT.my));
    preFM[2]=((quat[0]*_jr3._initT.my) - (quat[1]*_jr3._initT.mz) + (quat[3]*_jr3._initT.mx));
    preFM[3]=((quat[0]*_jr3._initT.mz) + (quat[1]*_jr3._initT.my) - (quat[2]*_jr3._initT.mx));
    FM[0]=((preFM[0]*quatC[0]) - (preFM[1]*quatC[1]) - (preFM[2]*quatC[2])) - (preFM[3]*quatC[3]);
    FM[1]=((preFM[0]*quatC[1]) + (preFM[1]*quatC[0]) + (preFM[2]*quatC[3])) - (preFM[3]*quatC[2]); //mx
    FM[2]=((preFM[0]*quatC[2]) - (preFM[1]*quatC[3]) + (preFM[2]*quatC[0])) + (preFM[3]*quatC[1]); //my
    FM[3]=((preFM[0]*quatC[3]) + (preFM[1]*quatC[2]) - (preFM[2]*quatC[1])) + (preFM[3]*quatC[0]); //mz

}

/************************************************************************/
void InSrPort::AxesTransform2(){
    /**
     * Transformation matrix between TEO_body_axes (world) and Jr3_axes
     * with horizontal tray (waiter)
    **/

    _tray._F.fx = + FF[3];
    _tray._F.fy = + FF[2];
    _tray._F.fz = + FF[1];
    _tray._M.mx = + FM[3];
    _tray._M.my = + FM[2];
    _tray._M.mz = + FM[1];
}

/************************************************************************/
void InSrPort::ZMPcomp(){
    /**
     * Bottle ZMP measurements
    **/

    if (_tray._F.fz==0)    {
        _tray._zmp.x_zmp = _tray._zmp.x_zmp; // Metros
        _tray._zmp.y_zmp = _tray._zmp.y_zmp; // Metros
    }else{
        _tray._zmp.x_zmp = ((- (_tray._M.my) / (_tray._F.fz)) - _d); // Metros
        _tray._zmp.y_zmp = ((_tray._M.mx) / _tray._F.fz); // Metros
    }

    if (_tray._zmp.x_zmp>0.075)    { //limitando el maximo ZMP en X positivo
        _tray._zmp.x_zmp = 0.075;
    }    if (_tray._zmp.x_zmp<-0.075)    {//limitando el maximo ZMP en X negativo
        _tray._zmp.x_zmp = -0.075;
    }

    if (_tray._zmp.y_zmp>0.075)    {//limitando el maximo ZMP en Y positivo
        _tray._zmp.y_zmp = 0.075;
    }    if (_tray._zmp.y_zmp<-0.075)    {//limitando el maximo ZMP en Y negativo
        _tray._zmp.y_zmp = -0.075;
    }

    _rzmp = sqrt(pow(_tray._zmp.x_zmp,2) + pow(_tray._zmp.y_zmp,2));

    //cout << "ZMP: [" << _tray.xzmp << ", " << _tray.yzmp << "]" << endl;
}

/************************************************************************/
void InSrPort::LIPM3d(){
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
        desireX[0] = currentX[0] + _tray._zmp.x_zmp; // new X position
        desireX[1] = currentX[1] + _tray._zmp.y_zmp; // new Y position
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
    CD_DEBUG_NO_HEADER("%f ",_tray._zmp.x_zmp);
    CD_DEBUG_NO_HEADER("%f ",_tray._zmp.y_zmp);
    CD_DEBUG_NO_HEADER("%f ",_rWorkSpace);
    CD_DEBUG_NO_HEADER("\n ");


    if( ! iPositionControl->positionMove( desireQ.data() ))        {
        CD_WARNING("setPositions failed, not updating control this iteration.\n");
    }

    return;

}

/************************************************************************/
void InSrPort::saveToFile()
{

    ofstream out;
    if(iteration==1) {
        out.open("cabeza.txt",ios::trunc);
    }
    else {
        out.open("cabeza.txt",ios::app);
    }
    out <<  _tray._zmp.x_zmp << " " <<  _tray._zmp.y_zmp << " " << endl;
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
}  // namespace teo


