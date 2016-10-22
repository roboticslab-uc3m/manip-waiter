// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WaiterDialogueManager.hpp"

namespace teo
{

/************************************************************************/

bool WaiterDialogueManager::configure(yarp::os::ResourceFinder &rf) {

    std::string language = rf.check("language",yarp::os::Value(DEFAULT_LANGUAGE),"language to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("WaiterDialogueManager options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--language (default: \"%s\")\n",language.c_str());
    }
    printf("WaiterDialogueManager using language: %s\n",language.c_str());

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    //-----------------OPEN LOCAL PORTS------------//
    outCmdPortHead.open("/waiterDialMan/Manip/command:o");
    outCmdPortManip.open("/waiterDialMan/Head/command:o");
    outTtsPort.open("/waiterDialMan/tts-en:o");
    inSrPort.open("/waiterDialMan/speechRecognition:i");
    stateMachine.setOutCmdPortHead(&outCmdPortHead);
    stateMachine.setOutCmdPortManip(&outCmdPortManip);
    stateMachine.setOutTtsPort(&outTtsPort);
    stateMachine.setInSrPort(&inSrPort);
    while(1){
        if(outTtsPort.getOutputCount() > 0) break;
        printf("Waiting for \"/waiterDialogueManager/tts:o\" to be connected to something...\n");
        yarp::os::Time::delay(0.5);
    }
    stateMachine.setLanguage(language);
    stateMachine.start();
    return true;
}

/************************************************************************/
double WaiterDialogueManager::getPeriod() {
    return 2.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool WaiterDialogueManager::updateModule() {
    printf("StateMachine in state [%d]. WaiterDialogueManager alive...\n", stateMachine.getMachineState());
    return true;
}

/************************************************************************/

bool WaiterDialogueManager::interruptModule() {
    printf("WaiterDialogueManager closing...\n");
    inSrPort.interrupt();
    outTtsPort.interrupt();
    stateMachine.stop();
    inSrPort.close();
    outTtsPort.close();
    return true;
}

/************************************************************************/

}  // namespace teo
