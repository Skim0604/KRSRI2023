#include <ax12.h>
#include <BioloidController.h>
#include "nuke.h"


void setup(){
    pinMode(0,OUTPUT);
    // setup IK
    setupIK();
    gaitSelect(RIPPLE);
    // setup serial
    Serial.begin(38400);

    // wait, then check the voltage (LiPO safety)
    delay (1000);
    float voltage = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
    Serial.print ("System Voltage: ");
    Serial.print (voltage);
    Serial.println (" volts.");
    if (voltage < 10.0)
        while(1);

    // stand up slowly
    bioloid.poseSize = 12;
    bioloid.readPose();
    doIK();
    bioloid.interpolateSetup(1000);
    while(bioloid.interpolating > 0){
        bioloid.interpolateStep();
        delay(3);
    }
}

/* this is the main loop, it repeats over and over again */
void loop(){
  // take commands from a commander or computer
  /*
  if(command.ReadMsgs() > 0){
    if(command.buttons&BUT_R3){ gaitSelect(RIPPLE);}
    if(command.buttons&BUT_L4){ gaitSelect(AMBLE);}
    digitalWrite(0,HIGH-digitalRead(0));
    //Xspeed = (2*(command.walkV));
    if((command.buttons&BUT_LT) > 0)
      Yspeed = (command.walkH);
    else
      Rspeed = -(command.walkH)/100.0;
    if(Rspeed > 0.5){
      //gaitSelect(AMBLE);
      Xspeed = command.walkH;
    }else if(currentGait == AMBLE){
      Xspeed = 4*command.walkV;
    }else{
      //gaitSelect(RIPPLE);
      Xspeed = 2*command.walkV;
    }
    
    bodyRotY = (((float)command.lookV))/250.0;
    if((command.buttons&BUT_RT) > 0)
      bodyRotX = ((float)command.lookH)/250.0;
    else
      bodyRotZ = ((float)command.lookH)/250.0;
  }
 */

 
  gaitSelect(RIPPLE_LEFT);
  Xspeed = 90;

  
  // if our previous interpolation is complete, recompute the IK
  if(bioloid.interpolating == 0){
    doIK();
    bioloid.interpolateSetup(tranTime);
  }

  // update joints
  bioloid.interpolateStep();

}
