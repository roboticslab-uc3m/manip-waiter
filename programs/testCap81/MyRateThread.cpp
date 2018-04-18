#include "MyRateThread.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <iomanip>

#include <fstream>

/************************************************************************/
bool MyRateThread::threadInit(){
    /**     * Generating Initial Trajectory - Waiter Pose *    **/

    fp = fopen("../data_zmp_bottle.csv","w+");

    leftArmIEncoders->getAxes(&numRobotJoints); // getting the number of joint on left-arm
    CD_INFO("numRobotJoints: %d.\n",numRobotJoints);

    /** ----- generate initial movement --------------- **/
    //leftArmIPositionControl2->setPositionMode();
    printf("begin MOVE TO START POSITION\n");
    double initpos[7] = {-30,0,0,-90,0,30,0};
    leftArmIPositionControl2->positionMove(initpos);

    /*
    //posicionamiento temporal hasta arreglar set poss
    iPositionControl->positionMove(0,-30);
    iPositionControl->positionMove(1,0);
    iPositionControl->positionMove(2,0);
    iPositionControl->positionMove(3,-90);
    iPositionControl->positionMove(4,0);
    iPositionControl->positionMove(5,30);*/
    yarp::os::Time::delay(10);  // provisional !!!!!!!!!!!!!!!!!!!!!!!!!!!!
    bool done = false;
    while( ! done )    {
        yarp::os::Time::delay(0.5);
        leftArmIPositionControl2->checkMotionDone(&done);
        printf(".");
        fflush(stdout);
    }
    printf("end MOVE TO START POSITION\n");

    /** ---- designate initial position --------------- **/
    if ( ! leftArmIEncoders->getEncoders( beforeQ.data() ) )    {
        CD_WARNING("[Error] getEncoders failed, not updating control this iteration.\n");
        return false;    }
    /** --------------------------------------------------- **/

    /** ----- set NEW ref speed --------------- **/
    double initspe[7] = {10.0,10.0,10.0,10.0,10.0,10.0,0.0};
    if(!leftArmIPositionControl2->setRefSpeeds(initspe)){
        CD_WARNING("[Error] Problems setting reference speed on left-arm joints.\n");
        return false;
    }
    /** --------------------------------------------------- **/

    /** ----- set NEW ref accelaration --------------- **/
    double initacc[7] = {10.0,10.0,10.0,10.0,10.0,10,0.0};
    if(!leftArmIPositionControl2->setRefAccelerations(initacc)){
        CD_WARNING("[Error] Problems setting reference acceleration on left-arm joints.\n");
        return false;
    }
    /** --------------------------------------------------- **/

    //fprintf(fp,"fx,fy,fz,Mx,My,Mz,Xzmp,Yzmp,Rzmp,CX3,CX4,CX5,CX6,DX3,DX4,DX5,DX6,TX1,TY1,TX2,TY2,initT,currT,diffT");
    fprintf(fp,"Xzmp,Yzmp");

    yarp::os::Time::delay(3);
    return true;
}

/************************************************************************/
void MyRateThread::run(){ // main process
    ReadSensor();
    AxesTransform();
    ZMPcomp();
}

/************************************************************************/
void MyRateThread::ReadSensor(){ //recogida de informacion de los sensores de las muñecas

    //port2.read(_sensor2.bottle); // si fuera necesario
    port3.read(_sensor3.bottle);
    IMU.read(_imu.bottle);

    /** Force-Sensors **/
/*    // si fuera necesario
    _sensor2._initF.fx = _sensor2.bottle.get(0).asDouble(); // Force vector - Right Arm
    _sensor2._initF.fy = _sensor2.bottle.get(1).asDouble();
    _sensor2._initF.fz = _sensor2.bottle.get(2).asDouble();
    _sensor2._initT.mx = _sensor2.bottle.get(3).asDouble(); // Torque vector - Right Arm
    _sensor2._initT.my = _sensor2.bottle.get(4).asDouble();
    _sensor2._initT.mz = _sensor2.bottle.get(5).asDouble();
*/
    _sensor3._initF.fx = _sensor3.bottle.get(0).asDouble(); // Force vector - Left Arm
    _sensor3._initF.fy = _sensor3.bottle.get(1).asDouble();
    _sensor3._initF.fz = _sensor3.bottle.get(2).asDouble();
    _sensor3._initT.mx = _sensor3.bottle.get(3).asDouble(); // Torque vector - Left Arm
    _sensor3._initT.my = _sensor3.bottle.get(4).asDouble();
    _sensor3._initT.mz = _sensor3.bottle.get(5).asDouble();

    /** Inertial-Sensor **/

    _imu.ang_x = _imu.bottle.get(0).asDouble(); // Angulo en X [deg]
    _imu.ang_y = _imu.bottle.get(1).asDouble(); // Angulo en Y [deg]
    _imu.ang_z = _imu.bottle.get(2).asDouble(); // Angulo en Z [deg]
    _imu.acc_x = _imu.bottle.get(3).asDouble(); //Linear acceleration in X [m/s^2]
    x_sensor.push_front(_imu.acc_x);
    x_sensor.pop_back();
    _imu.acc_y = _imu.bottle.get(4).asDouble(); //Linear acceleration in Y [m/s^2]
    y_sensor.push_front(_imu.acc_y);
    y_sensor.pop_back();
    _imu.acc_z = _imu.bottle.get(5).asDouble(); //Linear acceleration in Z [m/s^2]
    z_sensor.push_front(_imu.acc_z);
    z_sensor.pop_back();
    _imu.spd_x=_imu.bottle.get(6).asDouble(); // Velocidad angular en X [deg/s]
    _imu.spd_y=_imu.bottle.get(7).asDouble(); // Velocidad angular en Y [deg/s]
    _imu.spd_z=_imu.bottle.get(8).asDouble(); // Velocidad angular en Z [deg/s]
    _imu.mag_x=_imu.bottle.get(9).asDouble(); // Campo magnetico en X
    _imu.mag_y=_imu.bottle.get(10).asDouble(); // Campo magnetico en Y
    _imu.mag_z=_imu.bottle.get(11).asDouble(); // Campo magnetico en Z
}

/************************************************************************/
void MyRateThread::AxesTransform(){
    //Transformation matrix between TEO_body_axes (world) and Jr3_axes with horizontal tray (waiter)
    _tray._F.fx = + _sensor3._initF.fx;
    _tray._F.fy = - _sensor3._initF.fy;
    _tray._F.fz = + _sensor3._initF.fz;
    _tray._M.mx = + _sensor3._initT.mx;
    _tray._M.my = - _sensor3._initT.my;
    _tray._M.mz = + _sensor3._initT.mz;
}

/************************************************************************/
void MyRateThread::ZMPcomp(){ // Cálculo del ZMP de la botella
    /** ZMP Equations **/

    _tray._zmp.X = -((- (_tray._M.my) / (_tray._F.fz)) + _d); // Milimetros
    _tray._zmp.Y = ((_tray._M.mx) / _tray._F.fz); // Milimetros

    if (_tray._zmp.X>0.075)    { //limitando el maximo ZMP en X positivo
        _tray._zmp.X = 0.075;
    }
    if (_tray._zmp.X<-0.075)    {//limitando el maximo ZMP en X negativo
        _tray._zmp.X = -0.075;
    }

    if (_tray._zmp.Y>0.075)    {//limitando el maximo ZMP en Y positivo
        _tray._zmp.Y = 0.075;
    }
    if (_tray._zmp.Y<-0.075)    {//limitando el maximo ZMP en Y negativo
        _tray._zmp.Y = -0.075;
    }

    cout << "ZMP: [" << _tray._zmp.X << ", " << _tray._zmp.Y << "]" << endl;
    //cout << "ZMP: [" << _sensor3.fx << ", " << _sensor3.fy << ", " << _sensor3.fz << "]" << endl;

}

/************************************************************************/
void MyRateThread::saveToFile(){
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
//    cout << "ZMP: [" << _tray._zmp.X << "\t, " << _tray._zmp.Y << "]" << endl;
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
    fprintf(fp,",%.2f", _tray._zmp.X);
    fprintf(fp,",%.2f", _tray._zmp.Y);
//----------------------------------------------
/*   fprintf(fp,"\n%.2f", _jr3._initF.fx);
    fprintf(fp,",%.2f", _jr3._initF.fy);
    fprintf(fp,",%.2f", _jr3._initF.fz);
    fprintf(fp,",%.2f", _jr3._initT.mx);
    fprintf(fp,",%.2f", _jr3._initT.my);
    fprintf(fp,",%.2f", _jr3._initT.mz);
    fprintf(fp,",%.2f", _tray._zmp.X);
    fprintf(fp,",%.2f", _tray._zmp.Y);
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
    fprintf(fp,",%.2f", _diff_time);*/
//----------------------------------------------
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
