#ifndef _ratethread_H_
#define _ratethread_H_

#include "LIPM2d.h"
#include "pid.h"
#include "global.h"

static FILE *fp;

yarp::os::Port portft0;
yarp::os::Port portft1;
yarp::os::Port portImu;

/** Left Leg Device */
yarp::dev::PolyDriver leftLegDevice;
/** Left Leg ControlMode2 Interface */
yarp::dev::IControlMode2 *leftLegIControlMode2;
/** Left Leg PositionControl2 Interface */
yarp::dev::IPositionControl2 *leftLegIPositionControl2; // para control en posicion
/** Left Leg VelocityControl2 Interface */
yarp::dev::IVelocityControl2 *leftLegIVelocityControl2; // para control en velocidad

/** Right Leg Device */
yarp::dev::PolyDriver rightLegDevice;
/** Right Leg ControlMode2 Interface */
yarp::dev::IControlMode2 *rightLegIControlMode2;
/** Right Leg PositionControl2 Interface */
yarp::dev::IPositionControl2 *rightLegIPositionControl2; // para control en posicion
/** Right Leg VelocityControl2 Interface */
yarp::dev::IVelocityControl2 *rightLegIVelocityControl2; // para control en velocidad

/** Trunk Device */
yarp::dev::PolyDriver trunkDevice;
/** Trunk ControlMode2 Interface */
yarp::dev::IControlMode2 *trunkIControlMode2;
/** Trunk PositionControl2 Interface */
yarp::dev::IPositionControl2 *trunkIPositionControl2; // para control en posicion
/** Trunk VelocityControl2 Interface */
yarp::dev::IVelocityControl2 *trunkIVelocityControl2; // para control en velocidad

class MyRateThread : public RateThread
{
public:
    MyRateThread() : RateThread(dt*1000.0) {    //Conversion to [ms]

        x_sensor.resize(samples); // tamaño del vector en funcion del numero de muestras para promediar con el filtro
        y_sensor.resize(samples);
        z_sensor.resize(samples);

        n = 1;
        m = 0;
        e = 0.03225;  // Loli 0.0194
        sum_j = 0.0;
        sum_l = 0.0;
        offs_x_j = 0.0;
        offs_x_l = 0.0;
        offs_y = 0.0;
        X = 0.0;
        _angulo = 0.0; // El angulo es 0 para inclinar el robot hasta -10 grados
        _num = 1;
        _angulo2 = 0.0;

        Xzmp_imu=0.0; // inicializacion del ZMP IMU
        Yzmp_imu=0.0;

        //Xzmp_b=0.0; // inicializacion del ZMP de Jaunlo
        //Yzmp_b=0.0;
        //Xzmp_off = 0.0; //inicializacion del ZMP con offset corregido de Juanlo

        seno_x = 0.0;
        seno_z = 0.0;
        coseno_z = 0.0;
        radianes = 0.0;
        ddx=0.0;
        ddy=0.0;
        ddz=9.8;
        X2 = 0.0;
        offs_x_l2 = 0.0;
        sum_l2 = 0.0;
        zmp_ref = 0.0;
        _angle_ref_1 = 0.0;
        _angle_ref = 0.0;
    }

    void run()
    {
            //printf("[success] entrando en ratethread -> run\n");
/*            // Test escalon con rampa (fig. 9 y 12) // generacion del ZMP_ref para test FT sensor
            if (n <= 300){zmp_ref = 0.0;}
            else if (n >= 300 && n <= 330){zmp_ref = (0.1/30)*n - 1;} // variar desde 0.01 a 0.09
            else {zmp_ref = zmp_ref;}
*/
            getInitialTime();
            
            readSensors(); // calculo del ZMP_FT
            zmpComp(); // calculo del ZMP_FT
            
            //ControlJoints(); // Calculo y generacion de la actuacion en funcion del ZMP_ref
            //setJoints(); // Generacion de la actuacion en funcion del ZMP_ref // añadido a ControlJoints()

            printData();
            cout << endl << "Press ENTER to exit..." << endl;
            cout << "*******************************" << endl << endl;
            saveInFileCsv();
            n++;
            cout << n << endl << endl;
            getCurrentTime();

    }

    void getInitialTime()
    {
        if (n==1){init_time = Time::now();}
        init_loop = Time::now();
        it_time = init_loop - it_prev;
        it_prev = init_loop;
    }

    void getCurrentTime()
    {
        act_time = Time::now() - init_time;
        act_loop = Time::now() - init_loop;
    }

    void readSensors(){

        Bottle ch0;
        Bottle ch1;
        Bottle imu;

        portft0.read(ch0); // lectura del sensor JR3 ch0
        portft1.read(ch1); // lectura del sensor JR3 ch1
        portImu.read(imu); // lectura del sensor IMU

        //--- FT-Sensor
        _fx0 = ch0.get(0).asDouble();
        _fy0 = ch0.get(1).asDouble();
        _fz0 = ch0.get(2).asDouble();
        _mx0 = ch0.get(3).asDouble();
        _my0 = ch0.get(4).asDouble();
        _mz0 = ch0.get(5).asDouble();

        _fx1 = ch1.get(0).asDouble();
        _fy1 = ch1.get(1).asDouble();
        _fz1 = ch1.get(2).asDouble();
        _mx1 = ch1.get(3).asDouble();
        _my1 = ch1.get(4).asDouble();
        _mz1 = ch1.get(5).asDouble();

        //--- Inertial-Sensor
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

    void zmpComp(){

        //ZMP Equations : Double Support - FT

/*      //Con el coeficiente 1000 - unidades en milimetros
        _xzmp0 = -(((_my0/10) + e*_fx0)*1000) / _fz0; // xzmp0 in [mm]
        _yzmp0 = (((_mx0/10) + e*_fy0)*1000) / _fz0; // yzmp0 in [mm]

        _xzmp1 = -(((_my1/10) + e*_fx1)*1000) / _fz1; // xzmp1 in [mm]
        _yzmp1 = (((_mx1/10) + e*_fy1)*1000) / _fz1; // yzmp1 in [mm]
*/

        //Sin el coeficiente 1000 - unidades en metros
        _xzmp0_ft = -(((_my0/10) + e*_fx0)) / _fz0; // xzmp0_ft in [m] derecho
        //_yzmp0_ft = (((_mx0/10) + e*_fy0)) / _fz0; // yzmp0_ft in [m]
        _xzmp1_ft = -(((_my1/10) + e*_fx1)) / _fz1; // xzmp1_ft in [m] izquierdo
        //_yzmp1_ft = (((_mx1/10) + e*_fy1)) / _fz1; // yzmp1_ft in [m]

        _xzmp01_ft = (_xzmp0_ft * _fz0 + _xzmp1_ft * _fz1) / (_fz0 + _fz1); // xzmp_ft in [m] robot
        //_yzmp01_ft = (_yzmp0_ft * _fz0 + _yzmp1_ft * _fz1) / (_fz0 + _fz1); // yzmp_ft in [m]

        // OFFSET IMU - eliminando el offset de ddx_robot (aceleracion en X)
        if (n >=100 && n < 250){
            sum_j = ddx_robot + sum_j;
            offs_x_j = sum_j / (n-99);
            printf("offs = %f\n", offs_x_j);
        }

        // OFFSET FT - eliminando el offset de _xzmp01_ft (ZMP de ambos sensores en X)
        if (n >=1 && n < 50){
            sum_l = _xzmp01_ft + sum_l;
            offs_x_l = sum_l / n;
            printf("offs = %f\n", offs_x_l);
        }

        Xzmp_ft  = _xzmp01_ft - offs_x_l;
        //Yzmp_ft = _yzmp01_ft - offs_y;

        if ((_xzmp01_ft != _xzmp01_ft) || (_yzmp01_ft != _yzmp01_ft)){
            printf ("Warning: No zmp data\n");
        }

        ddx_robot = ddx_robot - offs_x_j;
        //ZERO MOMENT POINT COMPUTATION - IMU
        Xzmp_imu = Xcom - (Zcom / ddz_robot) * ddx_robot; //ZMP X coordinate [m]
        //Yzmp_imu = Ycom - (Zcom / ddz_robot) * ddy_robot; //ZMP Y coordinate [m]

    }

    void ControlJoints(){

        //DETERMINING STRATEGY
        lin_vel = x_sensor.at(0) * dt;
        w = sqrt(G / Zcom);
        capture_point = (lin_vel / w) + (Xzmp_imu);

        //PID
        actual_value = Xzmp_imu;
        if (n==275){ //Get initial position as setpoint [cm]
            setpoint = Xzmp_imu;
            printf("ZMP REF = %f\n", setpoint);
        }

        Xzmp_total = (Xzmp_ft + Xzmp_imu)/2;

        //setpoint = 0;
        pid_output_ankle = pidcontroller_ankle->calculate(setpoint, Xzmp_total); // antes actual_value
        pid_output_hip = pidcontroller_hip->calculate(setpoint, Xzmp_total);

/*        //-- EVALUACION DLIPM MODELO
        _eval_Ctrl.model(Xzmp_ft,zmp_ref);

        ka = 0.25 * zmp_ref + 9.95;

        _angle_ref_1 = (zmp_ref*(-G))/ (L*(ka-G));
        _angle_ref =  -_eval_Ctrl.y + _angle_ref_1;
*/

    // version hip strategy - solo es posible esta opcion
    if(n>=300)    {
        cout << endl << "HIP STRATEGY\n"  << endl;

        leftLegIVelocityControl2->velocityMove(4, -pid_output_ankle);     //Left Ankle
        Time::delay(0.005);
        rightLegIVelocityControl2->velocityMove(4, -pid_output_ankle);    //Right Ankle
        Time::delay(0.005);
        trunkIVelocityControl2->velocityMove(1, pid_output_hip);        //Hip
        Time::delay(0.005);
    }

       // version ankle o hip strategy - ambas son posibles
    if(n>=300)    {
        //-- IMU VEL CONTROL
        if (capture_point < 0.3 && capture_point > -0.3)      ///Ankle strategy
        {
            cout << endl << "ANKLE STRATEGY\n"  << endl;

            leftLegIVelocityControl2->velocityMove(4, -pid_output_ankle);     //Left Ankle
            Time::delay(0.005);
            rightLegIVelocityControl2->velocityMove(4, -pid_output_ankle);    //Right Ankle
            Time::delay(0.005);
            trunkIVelocityControl2->velocityMove(1, 0); // no tengo claro el cero        //Hip
            Time::delay(0.005);
        }
        else                                                    ///Hip strategy
        {
            cout << endl << "HIP STRATEGY\n"  << endl;

            leftLegIVelocityControl2->velocityMove(4, -pid_output_ankle);     //Left Ankle
            Time::delay(0.005);
            rightLegIVelocityControl2->velocityMove(4, -pid_output_ankle);    //Right Ankle
            Time::delay(0.005);
            trunkIVelocityControl2->velocityMove(1, pid_output_hip);        //Hip
            Time::delay(0.005);

        }
    }

    /*        //-- ANKLE STRATEGY
    rightLegIPositionControl2->positionMove(4, _angle_ref); // position in degrees
    leftLegIPositionControl2->positionMove(4, _angle_ref);
*/

    /*        //-- HIP STRATEGY
    posRightLeg->positionMove(5, -angle_y); // axial ankle Right Leg
    posRightLeg->positionMove(1, -angle_y); // axial hip Right Leg
    posLeftLeg->positionMove(5, angle_y); // axial ankle Left Leg
    posLeftLeg->positionMove(1, angle_y); // axial hip Left Leg
    */

    /*        //-- ANKLE VEL STRATEGY
    velRightLeg->velocityMove(4, -vel); // velocity in degrees per second
    velLeftLeg->velocityMove(4, -vel);
    */
}



/*    // añadido a ControlJoints()
   void setJoints(){

        //-- IMU VEL CONTROL
   if(n>300)    {
        if (capture_point < 0.12 && capture_point > -0.12)      //Ankle strategy
        {
            cout << endl << "ANKLE STRATEGY\n"  << endl;

            leftLegIVelocityControl2->velocityMove(4, -pid_output_ankle);     //Left Ankle
            Time::delay(0.005);
            rightLegIVelocityControl2->velocityMove(4, -pid_output_ankle);    //Right Ankle
            Time::delay(0.005);
            //trunkIVelocityControl2->velocityMove(1, pid_output_hip);         //Hip
            //Time::delay(0.005);
        }
        else                                                    //Hip strategy
        {
            cout << endl << "HIP STRATEGY\n"  << endl;

            leftLegIVelocityControl2->velocityMove(4, -pid_output_ankle);     //Left Ankle
            Time::delay(0.005);
            rightLegIVelocityControl2->velocityMove(4, -pid_output_ankle);    //Right Ankle
            Time::delay(0.005);
            trunkIVelocityControl2->velocityMove(1, pid_output_hip);        //Hip
            Time::delay(0.005);

        }
    }
                //-- ANKLE STRATEGY
        rightLegIPositionControl2->positionMove(4, _angle_ref); // position in degrees
        leftLegIPositionControl2->positionMove(4, _angle_ref);


                //-- HIP STRATEGY
        posRightLeg->positionMove(5, -angle_y); // axial ankle Right Leg
        posRightLeg->positionMove(1, -angle_y); // axial hip Right Leg
        posLeftLeg->positionMove(5, angle_y); // axial ankle Left Leg
        posLeftLeg->positionMove(1, angle_y); // axial hip Left Leg

//      velRightLeg->velocityMove(4, -vel); // velocity in degrees per second
//      velLeftLeg->velocityMove(4, -vel);
    }
*/

    void printData()
    {
/*        cout << endl << "El angulo 1 es: " << _angle_ref_a << endl;
        cout << endl << "El angulo 2 es: " << _angle_ref_b << endl;
        cout << endl << "El angulo 3 es: " << _angle_ref_c << endl;
        cout << endl << "El angulo 4 es: " << _angle_ref_d << endl;
        cout << endl << "El angulo 4 es: " << g << endl;
        cout << endl << "El angulo 4 es: " << ka << endl;
*/
        cout << endl << "El angulo imu es: " << ang_x << endl;
        cout << endl << "El ANKLE pid es: " << pid_output_ankle << endl;
        cout << endl << "El HIP pid es: " << pid_output_hip << endl;
        cout << endl << "El ZMP REF es: " << setpoint << endl;
        cout << endl << "El ZMP IMU es: " << Xzmp_imu << endl;
        cout << endl << "El ZMP FT es: " << Xzmp_ft << endl;
        cout << endl << "La capture_point es: " << capture_point << endl;
        //cout << endl << "ZMP_Error_Loli = ("<< _eval_x._zmp_error << ") [mm]" << endl;
        //cout << endl << "ZMP model es: " << _eval_x.y << endl;
        //cout << endl << "Num es: " << _num << endl;  _u_ref
        //cout << endl << "El _u_ref x es: " << _eval_x._u_ref << endl;
        //cout << endl << "El _angle_error x es: " << _eval_x._angle_error << endl;

    }

    void saveInFileCsv()
    {
        if(n==1){
            fprintf(fp,"Time,ang_x_imu,acc_x_imu,acc_y_imu,acc_z_imu,acc_x_robot,acc_y_robot,acc_z_robot,fx_ft0,fz_ft0,my_ft0,fx_ft1,fz_ft1,my_ft1,Xzmp_total,zmp_imu,Xzmp_ft,Xzmp_ft0,Xzmp_ft1,iter");
        }

        fprintf(fp,"\n%.2f", act_time);
        fprintf(fp,",%.10f", ang_x); // angulo x del sensor imu
        fprintf(fp,",%.10f", acc_x); // acc_x del sensor imu
        fprintf(fp,",%.10f", acc_y); // acc_y del sensor imu
        fprintf(fp,",%.10f", acc_z); // acc_z del sensor imu
        fprintf(fp,",%.10f", ddx_robot); // acc_x del robot
        fprintf(fp,",%.10f", ddy_robot); // acc_y del robot
        fprintf(fp,",%.10f", ddz_robot); // acc_z del robot
        fprintf(fp,",%.10f", _fx0); // f_x del sensor ft 0
        fprintf(fp,",%.10f", _fz0); // f_z del sensor ft 0
        fprintf(fp,",%.10f", _my0); // m_y del sensor ft 0
        fprintf(fp,",%.10f", _fx1); // f_x del sensor ft 1
        fprintf(fp,",%.10f", _fz1); // f_z del sensor ft 1
        fprintf(fp,",%.10f", _my1); // m_y del sensor ft 1
        //fprintf(fp,",%.10f", pid_output_ankle);
        //fprintf(fp,",%.10f", pid_output_hip);
        //fprintf(fp,",%.10f", setpoint);
        fprintf(fp,",%.8f", Xzmp_total);
        fprintf(fp,",%.8f", Xzmp_imu);
        fprintf(fp,",%.8f", Xzmp_ft);
        fprintf(fp,",%.8f", _xzmp0_ft);
        fprintf(fp,",%.8f", _xzmp1_ft);
        //fprintf(fp,",%.10f", capture_point);
        fprintf(fp,",%i", n);

    }

    void set(std::string _plane, PID *_pidcontroller_ankle, PID *_pidcontroller_hip)
    {
        plane = _plane;
        pidcontroller_ankle = _pidcontroller_ankle;
        pidcontroller_hip = _pidcontroller_hip;
    }

/*    //set - version JUANLO
    void set(std::string _plane, BufferedPort<Bottle> *_readPort, IVelocityControl *_velRightLeg,
             IVelocityControl *_velLeftLeg, IVelocityControl *_velTrunk, PID *_pidcontroller_ankle,
             PID *_pidcontroller_hip)
    {
        plane = _plane;
        velRightLeg = _velRightLeg;
        velLeftLeg = _velLeftLeg;
        velTrunk = _velTrunk;
        pidcontroller_ankle = _pidcontroller_ankle;
        pidcontroller_hip = _pidcontroller_hip;
        readPort = _readPort;
    }*/

private:
    int m, n;
    double _num; // Variable para jugar con las iteraciones (500 iteraciones -> 15 segundos)
    float e; // distance [m] between ground and sensor center

    LIPM2d _eval_Ctrl;

    //-- IMU variables
    double acc_x, acc_y, acc_z, ang_x, ang_y, ang_z, spd_x, spd_y, spd_z, mag_x, mag_y, mag_z; // IMU inputs
    //-- IMU LOW-FILTER variables & CONVERTION
    double ddx, ddy, ddz, ddx_robot, ddy_robot, ddz_robot; // additional acc variables

    double init_time, act_time, init_loop, act_loop, it_time, it_prev; // variables para los tiempos
    deque<double> x_sensor, y_sensor, z_sensor;

    //-- FT variables
    float _fx0, _fy0, _fz0, _mx0, _my0, _mz0; // F-T from sensor 0 [Fuerza en N y Pares en Nm*10]
    float _fx1, _fy1, _fz1, _mx1, _my1, _mz1; // F-T from sensor 1 [Fuerza en N y Pares en Nm*10]
    //-- FT LOW-FILTER variables
    float offs_x_j, offs_x_l, offs_x_l2; // zmp offset in initial time.
    float offs_y;
    float sum_j, sum_l, sum_l2;

    //-- ZMP variables
    float _xzmp0_ft, _yzmp0_ft; // ZMP sensor 0 (derecho)
    float _xzmp1_ft, _yzmp1_ft; // ZMP sensor 1 (izquierdo)
    float _xzmp01_ft, _yzmp01_ft; // ZMP robot (zmp_0 + zmp_0) metros
    float Xzmp_ft, Yzmp_ft; // Global ZMP-FT despues de filtrar
    double Xzmp_imu, Yzmp_imu, Xzmp_total; // Global ZMP-IMU despues de filtrar
    //, Xzmp_b, Yzmp_b, Xzmp_c, Xzmp_off; // x, y, z en [cm]

    //-- CONTROL/MOVEMENT variables
    float X, ref, angle_x, X2, zmp_ref, ref2, y2, _angle_ref, _angle_ref_1, _angle_ref_a, _angle_ref_b, _angle_ref_c, _angle_ref_d, ka;
    double _angulo, _angulo2, seno_x, seno_z, coseno_z, radianes; //Angulo para compensar las componentes de la aceleración en el modelo cart-table,
                              //Angulo2 para invertir los angulos negativos
    //-- SET & PID variables
    PID *pidcontroller_ankle, *pidcontroller_hip;
    std::string plane;
    double actual_value, setpoint, pid_output_ankle, pid_output_hip, initial_encoder;
    double capture_point, lin_vel, w;

    BufferedPort<Bottle> *readPort;
    IVelocityControl *velTrunk, *velRightLeg, *velLeftLeg;

};

#endif

