// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InCvPort.hpp"

namespace teo
{

/************************************************************************/
void InCvPort::setFollow(int value)
{
    follow = value;
}


/************************************************************************/
void InCvPort::setOutPort(yarp::os::Port * _pOutPort) {
    pOutPort = _pOutPort;
}


/************************************************************************/


void InCvPort::onRead(Bottle& b) {

    yarp::os::Bottle outputCartesian;

//------------------------SET LEFT ARM INITIAL POSITION------------------------
   if (a==0){

        //out.open("datawaiter.txt",ios::trunc);

        out.open("datawaiter.txt",ios::app);

        printf("MOVE TO START POSITION\n");
        /*iPositionControl->positionMove(0, -30);
        iPositionControl->positionMove(1, 0);
        iPositionControl->positionMove(2, 0);
        iPositionControl->positionMove(3, -90);
        iPositionControl->positionMove(4, 0);
        iPositionControl->positionMove(5, 30);*/
        double initpos[7] = {-30,0,0,-90,0,30,0};
        iPositionControl->positionMove(initpos);
        bool done = false;
        iteration=1;
        yarp::os::Time::delay(2);

/*        while( ! done )
        {
            yarp::os::Time::delay(2);
//            iPositionControl->checkMotionDone(&done);
            printf(".");
            fflush(stdout);
        }*/

        a=1;
    }


//------------------------READING INPUT MESSAGES FROM HEAD------------------------

    double x = b.get(0).asDouble(); //Data pxXpos
    double y = b.get(1).asDouble(); //Data pxYpos
    double angle = b.get(2).asDouble(); //Angle
    printf("%f %f %f\n",x,y,angle);

//------------------------CORRECTION OUTPUTS------------------------
    //double dist;

    if(angle>92 && angle<=110)  //Correction 01. Move arm Y left.
    {
      //  dist = 0.05 * cos( angle );
        if(( coordY + 0.01 ) <= 0.45 )
        {
            printf("THE BOTTLE GOES RIGHT \n");
            outputCartesian.addString("movj");
            outputCartesian.addDouble( 0.526 );
            outputCartesian.addDouble( coordY + 0.01);
            outputCartesian.addDouble( 0.309 );
            outputCartesian.addDouble( -1 );
            outputCartesian.addDouble( 0 );
            outputCartesian.addDouble( 0 );
            outputCartesian.addDouble( 90 );
            //Time::delay(0.1);
            coordY += 0.01;
        }
        else if(( coordY + 0.01 ) > 0.45 )
        {
            printf("BOTTLE FALL right!! \n");
        }

    }
    else if(angle>=70 && angle<88)  //Correction 02. Move arm Y right.
    {
        //dist = 0.05 * cos( angle );
        //printf("value coordYleft: %f", coordY);
        if(( coordY - 0.01 ) >= 0.25 )
        {
            printf("THE BOTTLE GOES LEFT \n");
            outputCartesian.addString("movj");
            outputCartesian.addDouble( 0.526 );
            outputCartesian.addDouble( coordY - 0.01);
            outputCartesian.addDouble( 0.309 );
            outputCartesian.addDouble( -1 );
            outputCartesian.addDouble( 0 );
            outputCartesian.addDouble( 0 );
            outputCartesian.addDouble( 90 );
            //Time::delay(1);
            coordY -= 0.01;
        }
        else if(( coordY - 0.01 ) < 0.25 )
        {
            printf("BOTTLE FALL left!! \n");
        }
    }
    else if( angle >= 88 && angle <= 92 )
    {
        printf("THE BOTTLE IS IN EQUILIBRIUM \n");
        outputCartesian.addString("movj");
        outputCartesian.addDouble( 0.526 );
        outputCartesian.addDouble( coordY );
        outputCartesian.addDouble( 0.309 );
        outputCartesian.addDouble( -1 );
        outputCartesian.addDouble( 0 );
        outputCartesian.addDouble( 0 );
        outputCartesian.addDouble( 90 );
    }

    if (outputCartesian.size() > 0)
        pOutPort->write(outputCartesian);

    out << x << " " << y << " " << angle << " " << coordY << " " << endl;
    //out.close();

/*

//------------------------FIRST TRIAL------------------------
    if (a==0){
        printf("MOVE TO START POSITION\n");
        iPositionControl->positionMove(0, -30);
        iPositionControl->positionMove(1, 0);
        iPositionControl->positionMove(2, 0);
        iPositionControl->positionMove(3, -90);
        iPositionControl->positionMove(4, 0);
        iPositionControl->positionMove(5, 30);

        Time::delay(5);
        a=1;
    }

    if (iteration==1){
        outputCartesian.addString("movj");
        outputCartesian.addDouble( 0.526 );
        outputCartesian.addDouble( 0.3 );
        outputCartesian.addDouble( 0.309 );
        outputCartesian.addDouble( -1 );
        outputCartesian.addDouble( 0 );
        outputCartesian.addDouble( 0 );
        outputCartesian.addDouble( 90 );

        iteration =0;
        Time::delay(5);
    }
    else{
        outputCartesian.addString("movj");
        outputCartesian.addDouble( 0.526 );
        outputCartesian.addDouble( 0.5 );
        outputCartesian.addDouble( 0.309 );
        outputCartesian.addDouble( -1 );
        outputCartesian.addDouble( 0 );
        outputCartesian.addDouble( 0 );
        outputCartesian.addDouble( 90 );

        iteration =1;
        Time::delay(5);
    }

    if (outputCartesian.size() > 0)
        pOutPort->write(outputCartesian);
/*
//------------------------READING INPUT MESSAGES FROM HEAD------------------------
    std::vector<double> info;

    double x = b.get(0).asDouble();
    double y = b.get(1).asDouble();
    double z = b.get(2).asDouble();

    for(size_t r=1; r<b.size();r++){
        info.push_back(b.get(r).asDouble());
        printf("The bottle is: %f \n", b.get(r).asDouble());
    }
    */
  /*
    iteration=1;

    if (iteration==1){
    printf("MOVE TO START POSITION\n");
    iPositionControl->positionMove(0, -30);
    iPositionControl->positionMove(1, 0);
    iPositionControl->positionMove(2, 0);
    iPositionControl->positionMove(3, -90);
    iPositionControl->positionMove(4, 0);
    iPositionControl->positionMove(5, 30);
    iteration =0;
    Time::delay(5);
    }

    if(degrees<90 && degrees>85){
        outputCartesian();
    } */

//------------------------CREATING THE CORRECTION ORDER------------------------


//------------------------OUTPUT MESSAGES------------------------
/* int i=0;

        outputCartesian.addString("movj");
        outputCartesian.addDouble( i );
        outputCartesian.addDouble( i+1 );
        outputCartesian.addDouble( i+2 );
        outputCartesian.addDouble( -1 );
        outputCartesian.addDouble( 0 );
        outputCartesian.addDouble( 0 );
        outputCartesian.addDouble( 90 );
        i++;

        if (outputCartesian.size() > 0)
            pOutPort->write(outputCartesian);

*/


/*
    a=0;
    c=0;
    iteration=1;

    for(c=0; c<3; c++){
        a=0;
        if (a==0) {
            printf("MOVE TO POSITION 01\n");
            iPositionControl->positionMove(0, -30);
            iPositionControl->positionMove(1, 40);
            iPositionControl->positionMove(2, 0);
            iPositionControl->positionMove(3, -70);
            iPositionControl->positionMove(4, -40);
            iPositionControl->positionMove(5, 10);
            iPositionControl->positionMove(6, 0);

            Time::delay(5);
            a=1;
        }

        if (a==1) {
            printf("MOVE TO POSITION 02\n");
            iPositionControl->positionMove(0, -10);
            iPositionControl->positionMove(1, 30);
            iPositionControl->positionMove(2, 0);
            iPositionControl->positionMove(3, -90);
            iPositionControl->positionMove(4, -30);
            iPositionControl->positionMove(5, 10);
            iPositionControl->positionMove(6, 0);

            Time::delay(5);
            a=2;
        }

        if (a==2) {
            printf("MOVE TO POSITION 03\n");
            iPositionControl->positionMove(0, -30);
            iPositionControl->positionMove(1, -10);
            iPositionControl->positionMove(2, 0);
            iPositionControl->positionMove(3, -70);
            iPositionControl->positionMove(4, 10);
            iPositionControl->positionMove(5, 10);
            iPositionControl->positionMove(6, 0);

            Time::delay(5);
            a=0;
        }
    }

    iPositionControl->positionMove(0, 0);
    iPositionControl->positionMove(1, 0);
    iPositionControl->positionMove(2, 0);
    iPositionControl->positionMove(3, 0);
    iPositionControl->positionMove(4, 0);
    iPositionControl->positionMove(5, 0);
    iPositionControl->positionMove(6, 0);

*/
/*

        double movementTime = yarp::os::Time::now() - movementStartTime;

        if( movementTime > DEFAULT_DURATION )
        {
    //        this->stopControl();
            return;
        }

        //-- Obtain current joint position
        std::vector<double> currentQ(numRobotJoints);
        if ( ! iEncoders->getEncoders( currentQ.data() ) )
        {
            CD_WARNING("getEncoders failed, not updating control this iteration.\n");
            return;
        }

        //-- Obtain desired Cartesian position and velocity.
        std::vector<double> desiredX, desiredXdot;
        trajectory.getX(movementTime, desiredX);
        trajectory.getXdot(movementTime, desiredXdot);

        //-- Apply control law to compute robot Cartesian velocity commands.
        std::vector<double> commandXdot;
        iCartesianSolver->fwdKinError(desiredX,currentQ, commandXdot);
        for(unsigned int i=0; i<6; i++)
        {
            commandXdot[i] *= -DEFAULT_GAIN;
            commandXdot[i] += desiredXdot[i];
        }

        //-- Compute joint velocity commands and send to robot.
        std::vector<double> commandQdot;
        if (! iCartesianSolver->diffInvKin(currentQ,commandXdot,commandQdot) )
        {
            CD_WARNING("diffInvKin failed, not updating control this iteration.\n");
            return;
        }

        for(int i=0;i<commandQdot.size();i++)
        {
            if( fabs(commandQdot[i]) > DEFAULT_QDOT_LIMIT)
            {
                CD_ERROR("diffInvKin too dangerous, STOP!!!.\n");
                for(int i=0;i<commandQdot.size();i++)
                    commandQdot[i] = 0;
                //setCurrentState(VOCAB_CC_NOT_CONTROLLING);
            }
        }

        CD_DEBUG_NO_HEADER("[MOVL] [%f] ",movementTime);
        for(int i=0;i<6;i++)
            CD_DEBUG_NO_HEADER("%f ",commandXdot[i]);
        CD_DEBUG_NO_HEADER("-> ");
        for(int i=0;i<numRobotJoints;i++)
            CD_DEBUG_NO_HEADER("%f ",commandQdot[i]);
        CD_DEBUG_NO_HEADER("[deg/s]\n");

        if( ! iVelocityControl->velocityMove( commandQdot.data() ) )
        {
            CD_WARNING("velocityMove failed, not updating control this iteration.\n");
        }

*/


/*
    switch ( follow ) {
        case 1: {//VOCAB_HELLO_TEO
            iPositionControl->positionMove(0, -30);
            iPositionControl->positionMove(1, -10);
            iPositionControl->positionMove(2, 0);
            iPositionControl->positionMove(3, -70);
            iPositionControl->positionMove(4, 10);
            iPositionControl->positionMove(5, 10);
            iPositionControl->positionMove(6, 0);
            break;
        }
        case 2: {//VOCAB_GO_TEO

        // j.movj()

            int a=0;
            if (a==0 && follow) {
                printf("MOVE TO POSITION 01\n");
                iPositionControl->positionMove(0, -30);
                iPositionControl->positionMove(1, 40);
                iPositionControl->positionMove(2, 0);
                iPositionControl->positionMove(3, -70);
                iPositionControl->positionMove(4, -40);
                iPositionControl->positionMove(5, 10);
                iPositionControl->positionMove(6, 0);

                Time::delay(3);
                a=1;
            }

            if (a==1 && follow) {
                printf("MOVE TO POSITION 02\n");
                iPositionControl->positionMove(0, -10);
                iPositionControl->positionMove(1, 30);
                iPositionControl->positionMove(2, 0);
                iPositionControl->positionMove(3, -90);
                iPositionControl->positionMove(4, -30);
                iPositionControl->positionMove(5, 10);
                iPositionControl->positionMove(6, 0);

                Time::delay(3);
                a=2;
            }

            if (a==2 && follow) {
                printf("MOVE TO POSITION 03\n");
                iPositionControl->positionMove(0, -30);
                iPositionControl->positionMove(1, -10);
                iPositionControl->positionMove(2, 0);
                iPositionControl->positionMove(3, -70);
                iPositionControl->positionMove(4, 10);
                iPositionControl->positionMove(5, 10);
                iPositionControl->positionMove(6, 0);

                Time::delay(3);
                a=0;
            }

            break;
        }
        case 3: {//VOCAB_WATER_PLEASE
            iPositionControl->positionMove(0, -40);
            iPositionControl->positionMove(1, -10);
            iPositionControl->positionMove(2, 0);
            iPositionControl->positionMove(3, -60);
            iPositionControl->positionMove(4, 10);
            iPositionControl->positionMove(5, 10);
            iPositionControl->positionMove(6, 0);
            break;
        }
        case 4: {//VOCAB_STOP_TEO
            iPositionControl->positionMove(0, 0);
            iPositionControl->positionMove(1, 0);
            iPositionControl->positionMove(2, 0);
            iPositionControl->positionMove(3, 0);
            iPositionControl->positionMove(4, 0);
            iPositionControl->positionMove(5, 0);
            iPositionControl->positionMove(6, 0);
            break;
        }
        default:
            break;
    }
*/

 /*
    if ( ! follow ) {
        iPositionControl->positionMove(0, 0);
        iPositionControl->positionMove(1, 0);
        iPositionControl->positionMove(2, 0);
        iPositionControl->positionMove(3, 0);
        iPositionControl->positionMove(4, 0);
        iPositionControl->positionMove(5, 0);
        iPositionControl->positionMove(6, 0);
        return;
    }

    int a=0;
    if (a==0 && follow) {
        printf("MOVE TO POSITION 01\n");
        iPositionControl->positionMove(0, -30);
        iPositionControl->positionMove(1, 40);
        iPositionControl->positionMove(2, 0);
        iPositionControl->positionMove(3, -70);
        iPositionControl->positionMove(4, -40);
        iPositionControl->positionMove(5, 10);
        iPositionControl->positionMove(6, 0);

        Time::delay(5);
        a=1;
    }

    if (a==1 && follow) {
        printf("MOVE TO POSITION 02\n");
        iPositionControl->positionMove(0, -10);
        iPositionControl->positionMove(1, 30);
        iPositionControl->positionMove(2, 0);
        iPositionControl->positionMove(3, -90);
        iPositionControl->positionMove(4, -30);
        iPositionControl->positionMove(5, 10);
        iPositionControl->positionMove(6, 0);

        Time::delay(5);
        a=2;
    }

    if (a==2 && follow) {
        printf("MOVE TO POSITION 03\n");
        iPositionControl->positionMove(0, -30);
        iPositionControl->positionMove(1, -10);
        iPositionControl->positionMove(2, 0);
        iPositionControl->positionMove(3, -70);
        iPositionControl->positionMove(4, 10);
        iPositionControl->positionMove(5, 10);
        iPositionControl->positionMove(6, 0);

        Time::delay(5);
        a=0;
    }
*/

}

/************************************************************************/





}  // namespace teo

