// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InCvPort.hpp"

namespace teo
{

/************************************************************************/

void InCvPort::onRead(Bottle& b) {
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

void InCvPort::setFollow(int value)
{
    follow = value;
}

/************************************************************************/

}  // namespace teo

