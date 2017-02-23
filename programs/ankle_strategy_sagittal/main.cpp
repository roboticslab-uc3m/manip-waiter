#include "global.h"
#include "ratethread.h"

//yarp::os::Port port0;
//yarp::os::Port port1;

void controlC (void){
    fclose(fp);
    port0.close();
    port1.close();
    exit(-1);
}

int main(void) {
    yarp::os::Network yarp;

    fp = fopen("../data.csv","w+");

 //   signal (SIGINT, (__sighandler_t) controlC );

    /** Check yarp network**/
    printf("Checking network...\n");
    if (!yarp.checkNetwork()) {
        printf("Please start a yarp name server first\n");
       return(-1);
    }
    printf("Network ok\n");

    /** Opening YARP ports**/
    port0.open("/jr3ch0:i");
    port1.open("/jr3ch1:i");
    yarp::os::Time::delay(0.5);
    yarp.connect("/jr3ch0:o","/jr3ch0:i");
    yarp::os::Time::delay(0.5);
    yarp.connect("/jr3ch1:o","/jr3ch1:i");

    /** SET CONFIG LEFT LEG **/
    yarp::os::Property optionsLeftLeg;
    optionsLeftLeg.put("device","remote_controlboard");
    optionsLeftLeg.put("remote","/teo/leftLeg");
    optionsLeftLeg.put("local","/loli/leftLeg");
    yarp::dev::PolyDriver devLeftLeg(optionsLeftLeg);
    if(!devLeftLeg.isValid()) {
        printf("TEO device not available.\n");
        devLeftLeg.close();
        yarp::os::Network::fini();
        return 1;
    }
    // Position control
    if (! devLeftLeg.view(posLeftLeg)) {
        printf("[warning] Problems acquiring robot IPositionControl leftLeg interface\n");
        return false;
    } else printf("[success] testTEO acquired robot IPositionControl leftLeg interface\n");
    // Velocity control
/**    if (! devLeftLeg.view(velLeftLeg)) {
        printf("[warning] Problems acquiring robot IVelocityControl leftLeg interface\n");
        return false;
    } else printf("[success] testTEO acquired robot IVelocityControl leftLeg interface\n");
**/
    /** SET CONFIG RIGHT LEG **/
    yarp::os::Property optionsRightLeg;
    optionsRightLeg.put("device","remote_controlboard");
    optionsRightLeg.put("remote","/teo/rightLeg");
    optionsRightLeg.put("local","/loli/rightLeg");

    yarp::dev::PolyDriver devRightLeg(optionsRightLeg);
    if(!devRightLeg.isValid()) {
        printf("TEO device not available.\n");
        devRightLeg.close();
        yarp::os::Network::fini();
        return 1;
    }
    // Position control
    if (! devRightLeg.view(posRightLeg)) {
        printf("[warning] Problems acquiring robot IPositionControl rightLeg interface\n");
        return false;
    } else printf("[success] testTEO acquired robot IPositionControl rightLeg inteface\n");
    // Velocity control
/**    if (!devRightLeg.view(velRightLeg)) {
        printf("[warning] Problems acquiring robot IVelocityControl rightLeg interface\n");
        return false;
    } else printf("[success] testTEO acquired robot IPositionControl rightLeg inteface\n");
**/

    /** SET MODE **/
    /** Position Mode **/
    printf("Set position mode Left Leg\n");
    posLeftLeg->setPositionMode();
    printf("Set position mode Right Leg\n");
    posRightLeg->setPositionMode();
    /** Velocity Mode **/
/**    printf("Set velocity mode Left Leg\n");
    velLeftLeg->setVelocityMode();
    printf("Set velocity mode Right Leg\n");
    velRightLeg->setVelocityMode();
**/

    /** POSITION CONTROL. SET LEFT LEG TO 0 **/
    posLeftLeg->positionMove(0,0);
    posLeftLeg->positionMove(1,0);
    posLeftLeg->positionMove(2,0);
    posLeftLeg->positionMove(3,0);
    posLeftLeg->positionMove(4,0);
    posLeftLeg->positionMove(5,0);
    printf("Left Leg :(0 0 0 0 0 0)\n");
    /** POSITION CONTROL. SET RIGHT LEG TO 0 **/
    posRightLeg->positionMove(0,0);
    posRightLeg->positionMove(1,0);
    posRightLeg->positionMove(2,0);
    posRightLeg->positionMove(3,0);
    posRightLeg->positionMove(4,0);
    posRightLeg->positionMove(5,0);
    printf("Right Leg :(0 0 0 0 0 0)\n");

    yarp::os::Time::delay(5);
    /** LOOP THREAD **/
    MyRateThread myRateThread;
    myRateThread.start();

    char c;
    do {
        c=getchar();
    } while(c != '\n');
    myRateThread.stop();
    port0.close();
    port1.close();
    devRightLeg.close();
    devLeftLeg.close();
    yarp::os::Time::delay(0.5);

}
