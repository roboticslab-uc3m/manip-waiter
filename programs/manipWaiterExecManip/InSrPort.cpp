// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InSrPort.hpp"

namespace teo
{

/************************************************************************/

void InSrPort::onRead(Bottle& b) {
    switch ( b.get(0).asVocab() ) {
        case VOCAB_HELLO_TEO:
            printf("hello teo\n");
            inCvPortPtr->setFollow(1);
            break;
        case VOCAB_GO_TEO:
            printf("go teo\n");
            inCvPortPtr->setFollow(2);
            break;
        case VOCAB_WATER_PLEASE:
            printf("water please\n");
            inCvPortPtr->setFollow(3);
            break;
        case VOCAB_STOP_TEO:
            printf("stop teo\n");
            inCvPortPtr->setFollow(4);
            break;
        default:
            break;
    }
}

/************************************************************************/

}  // namespace teo
