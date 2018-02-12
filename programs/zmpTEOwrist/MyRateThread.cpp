#include "MyRateThread.hpp"

/************************************************************************/
void MyRateThread::run(){// proceso principal
    ReadSensor();
    AxesTransform();
    ZMPcomp();
}

/************************************************************************/
void MyRateThread::ReadSensor(){ //recogida de informacion de los sensores de las muñecas

    port2.read(_sensor2.bottle);
    port3.read(_sensor3.bottle);
    IMU.read(_imu.bottle);

    /** Force-Sensors **/

    _sensor2.fx = _sensor2.bottle.get(0).asDouble(); // Force vector - Right Arm
    _sensor2.fy = _sensor2.bottle.get(1).asDouble();
    _sensor2.fz = _sensor2.bottle.get(2).asDouble();
    _sensor2.mx = _sensor2.bottle.get(3).asDouble(); // Torque vector - Right Arm
    _sensor2.my = _sensor2.bottle.get(4).asDouble();
    _sensor2.mz = _sensor2.bottle.get(5).asDouble();

    _sensor3.fx = _sensor3.bottle.get(0).asDouble(); // Force vector - Left Arm
    _sensor3.fy = _sensor3.bottle.get(1).asDouble();
    _sensor3.fz = _sensor3.bottle.get(2).asDouble();
    _sensor3.mx = _sensor3.bottle.get(3).asDouble(); // Torque vector - Left Arm
    _sensor3.my = _sensor3.bottle.get(4).asDouble();
    _sensor3.mz = _sensor3.bottle.get(5).asDouble();

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
    _tray.fx = + _sensor3.fz;
    _tray.fy = - _sensor3.fy;
    _tray.fz = + _sensor3.fx;
    _tray.mx = + _sensor3.mz;
    _tray.my = - _sensor3.my;
    _tray.mz = + _sensor3.mx;
}

/************************************************************************/
void MyRateThread::ZMPcomp(){ // Cálculo del ZMP de la botella
    /** ZMP Equations **/

    _tray.xzmp = -((- (_tray.my) / (_tray.fz)) + _d); // Milimetros
    _tray.yzmp = ((_tray.mx) / _tray.fz); // Milimetros

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

    cout << "ZMP: [" << _tray.xzmp << ", " << _tray.yzmp << "]" << endl;
    //cout << "ZMP: [" << _sensor3.fx << ", " << _sensor3.fy << ", " << _sensor3.fz << "]" << endl;

}
