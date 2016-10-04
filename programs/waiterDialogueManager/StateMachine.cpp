// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StateMachine.hpp"

namespace teo
{

/************************************************************************/

bool StateMachine::threadInit() {
    _machineState = 0;
    return true;
}

/************************************************************************/

void StateMachine::run() {
    while(!isStopping()) {
            if(_machineState==-1) {
              //ttsSay( yarp::os::ConstString("Sorry, I do not know what that is.") );
                ttsSay( notUnderstand );  //-- Sorry, I do not undestand
                _machineState=0;
            } else if(_machineState==0) {
              //ttsSay( yarp::os::ConstString("Please tell me.") );
                ttsSay( repeat );  //-- Please tell me
                _machineState=1;
            } else if(_machineState==1) {
                yarp::os::ConstString inStr = asrListen();
                // Blocking
                _inStrState1 = inStr;
                if( _inStrState1.find( hiTeo ) != yarp::os::ConstString::npos ) _machineState=2;  //-- Hi teo
                else if ( _inStrState1.find( goOnTeo ) != yarp::os::ConstString::npos ) _machineState=3;  //-- Go on teo
                else if ( _inStrState1.find( waterPlease) != yarp::os::ConstString::npos ) _machineState=4;  //-- Water please
                else if ( _inStrState1.find( stopNow ) != yarp::os::ConstString::npos ) _machineState=5;  //-- Stop now
                else _machineState=-1;
            } else if (_machineState==2) {
              //ttsSay( yarp::os::ConstString("Hi, I am teo, your waiter.") );
                ttsSay( hello );  //-- Hi, I am teo, your waiter
                yarp::os::Bottle cmd;
                cmd.addVocab(VOCAB_HELLO_TEO);
                outCmdPortHead->write(cmd);
                outCmdPortManip->write(cmd);
                _machineState=0;
            } else if (_machineState==3) {
              //ttsSay( yarp::os::ConstString("Are you thirsty.") );
                ttsSay( drink );  //-- Are you thirsty
                yarp::os::Bottle cmd;
                cmd.addVocab(VOCAB_GO_TEO);
                outCmdPortHead->write(cmd);
                outCmdPortManip->write(cmd);
                _machineState=0;
            } else if (_machineState==4) {
              //ttsSay( yarp::os::ConstString("Here you are.") );
                ttsSay( take );  //-- Here you are
                yarp::os::Bottle cmd;
                cmd.addVocab(VOCAB_WATER_PLEASE);
                outCmdPortHead->write(cmd);
                outCmdPortManip->write(cmd);
                _machineState=0;
            } else if (_machineState==5) {
              //ttsSay( yarp::os::ConstString("Okay, see you later aligator.") );
                ttsSay( finish );  //-- Okay, see you later aligator
                yarp::os::Bottle cmd;
                cmd.addVocab(VOCAB_STOP_TEO);
                outCmdPortHead->write(cmd);
                outCmdPortManip->write(cmd);
                _machineState=0;
            } else {
                ttsSay( yarp::os::ConstString("ANOMALY") );
                _machineState=0;
            }
        }

}

/************************************************************************/

void StateMachine::ttsSay(const yarp::os::ConstString& sayConstString) {
    yarp::os::Bottle bOut;
    bOut.addString(sayConstString);
    outTtsPort->write(bOut);
    printf("[StateMachine] Said: %s\n", sayConstString.c_str());
    return;
}

/************************************************************************/

yarp::os::ConstString StateMachine::asrListen() {
    yarp::os::Bottle* bIn = inSrPort->read(true);  // shouldWait
    printf("[StateMachine] Listened: %s\n", bIn->toString().c_str());
    return bIn->get(0).asString();
}

/************************************************************************/

int StateMachine::getMachineState() {
    return _machineState;
}

/************************************************************************/

void StateMachine::setInSrPort(yarp::os::BufferedPort<yarp::os::Bottle>* inSrPort) {
    this->inSrPort = inSrPort;
}

/************************************************************************/

void StateMachine::setOutCmdPortHead(yarp::os::Port* outCmdPort) {
    this->outCmdPortHead = outCmdPort;
}

/************************************************************************/

void StateMachine::setOutCmdPortManip(yarp::os::Port* outCmdPort) {
    this->outCmdPortManip = outCmdPort;
}

/************************************************************************/

void StateMachine::setOutTtsPort(yarp::os::Port* outTtsPort) {
    this->outTtsPort = outTtsPort;
}

/************************************************************************/

bool StateMachine::setLanguage(std::string language) {
    int z = 1;
    if("english" == language)
    {
        //-- recognition sentences
        hiTeo = std::string ("Hello TEO"); //state 2
        goOnTeo = std::string ("Go TEO"); //state 3
        waterPlease = std::string ("Water please"); //state 4
        stopNow = std::string ("Stop TEO"); //state 5
        //-- speak sentences
        notUnderstand = std::string("Sorry, I do not understand."); //state -1
        repeat = std::string("Please tell me."); //state 0
        hello = std::string("Hi, I am teo, your waiter."); //state 2
        drink = std::string("Are you thirsty."); //state 3
        take = std::string("Here you are."); //state 4
        finish = std::string("Okay, see you later aligator."); //state 5
        return true;
    }
    else if("spanish" == language)
    {
        //-- frases de reconociomiento
        hiTeo = std::string ("Hello TEO"); //state 2
        goOnTeo = std::string ("Go TEO"); //state 3
        waterPlease = std::string ("Water please"); //state 4
        stopNow = std::string ("Stop TEO"); //state 5
        /*
        hiTeo = std::string ("Hola TEO"); //state 2
        goOnTeo = std::string ("Continua TEO"); //state 3
        waterPlease = std::string ("Agua por favor"); //state 4
        stopNow = std::string ("Hasta luego"); //state 5
        */
        //-- frases del habla
        notUnderstand = std::string("Disculpe, no le he entendido."); //state -1
        repeat = std::string("Puede grepetirlo."); //state 0
        hello = std::string("Hola, me yamo TEO y soy un grobot camarero."); //state 2

        if (z == 1){
            drink = std::string("Mi especialidad es servir cervezas bien fresquitas. Quiere una cerveza."); //state 3
            take = std::string("Aqui tiene su botella."); //state 4
            z = 2;
        }
        if (z == 2){
            drink = std::string("Tambien puede ofrecerle otras bebidads. Que le apetece."); //state 3
            take = std::string("Por favor, sirvase."); //state 4
            z = 1;
        }

        finish = std::string("Nos vemos pronto."); //state 5

        return true;
    }
    else
    {
        printf("error! %s????\n",language.c_str());
        return false;
    }
}



/************************************************************************/

}  // namespace teo
