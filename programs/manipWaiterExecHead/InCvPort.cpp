// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InCvPort.hpp"

namespace teo
{

/************************************************************************/

void InCvPort::onRead(Bottle& b) {

/*    if ( ! follow ){
	iVelocityControl->velocityMove(0, 0.0);
    iVelocityControl->velocityMove(1, 0.0);
	return;
    }*/

    if (b.size() < 3) return;

    double x = b.get(0).asDouble();
    double y = b.get(1).asDouble();
    double z = b.get(2).asDouble();


    printf("%f %f %f\n",x,y,z);

    if( x > 385 ) {
        iVelocityControl->velocityMove(0, -6.0); //Id motor. Velocity [deg/s].
    }
    else if( x > 365 && x < 385 ) {
        iVelocityControl->velocityMove(0, -4.0);
    }
    else if( x > 345 && x < 365 ) {
        iVelocityControl->velocityMove(0, -2.0);
    }
    else if( x > 325 && x < 345 ) {
        iVelocityControl->velocityMove(0, -1.0);
    }
    else if( x > 315 && x < 325 ) {
        iVelocityControl->velocityMove(0, 0.0);
    }
    else if( x < 315 && x > 295 ) {
        iVelocityControl->velocityMove(0, 1.0);
    }
    else if( x < 295 && x > 275 ) {
        iVelocityControl->velocityMove(0, 2.0);
    }
    else if( x < 275 && x > 255 ) {
        iVelocityControl->velocityMove(0, 4.0);
    }
    else if( x < 255 ) {
        iVelocityControl->velocityMove(0, 6.0);
    }



    if( y > 185 ) {
        iVelocityControl->velocityMove(1, -5.0); //Id motor. Velocity [deg/s].
    }
    else if( y > 165 && y < 185) {
        iVelocityControl->velocityMove(1, -3.0);
    }
    else if( y > 145 && y < 165 ) {
        iVelocityControl->velocityMove(1, -1.0);
    }
    else if( y > 135 && y < 145 ) {
        iVelocityControl->velocityMove(1, 0.0);
    }
    else if( y < 135 && y > 115 ) {
        iVelocityControl->velocityMove(1, 1.0);
    }
    else if( y < 115 && y > 95 ) {
        iVelocityControl->velocityMove(1, 3.0);
    }
    else if( y < 95 ) {
        iVelocityControl->velocityMove(1, 5.0);
    }

}

/************************************************************************/

void InCvPort::setFollow(bool value)
{
    follow = value;
}

/************************************************************************/

}  // namespace teo

