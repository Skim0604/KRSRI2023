#include <ax12.h>
#include <BioloidController.h>
#include <Arduino.h>
#include "nuke.h"

/* min and max positions for each servo */
//int mins[] = {512, 205, 165, 358, 335, 220, 205, 512, 165, 358, 335, 220};
//int maxs[] = {818, 512, 665, 850, 800, 689, 512, 818, 665, 850, 800, 689};

int limitMax [] = {0, 850, 850, 850, 850,
                   618, 618, 618, 618,
                   618, 618, 618, 618
                  }; //680
//640
//643

int MID_POSS [] = {0, 512, 512, 512, 512,
                   512, 512, 512, 512,
                   512, 512, 512, 512
                  };

int limitMin [] = {0, 220, 220, 220, 220,
                   220, 220, 220, 220,
                   220, 220, 220, 220
                  }; //360
//320
//360

/* IK Engine */
BioloidController bioloid = BioloidController(1000000);
ik_req_t endpoints[LEG_COUNT];
float bodyRotX = 0;             // body roll (rad)
float bodyRotY = 0;             // body pitch (rad)
float bodyRotZ = 0;             // body rotation (rad)
int bodyPosX = 0;               // body offset (mm)
int bodyPosY = 0;               // body offset (mm)
int Xspeed;                     // forward speed (mm/s)
int Yspeed;                     // sideward speed (mm/s)
float Rspeed;                   // rotation speed (rad/s)

/* Gait Engine */
int gaitLegNo[LEG_COUNT];       // order to step through legs
ik_req_t gaits[LEG_COUNT];      // gait engine output
int pushSteps;                  // how much of the cycle we are on the ground
int stepsInCycle;               // how many steps in this cycle
int step;                       // current step
int tranTime;
int liftHeight;
float cycleTime;                // cycle time in seconds (adjustment from speed to step-size)

/* Setup the starting positions of the legs. */
void setupIK() {
  endpoints[RIGHT_FRONT].x = 0;
  endpoints[RIGHT_FRONT].y = 134;
  endpoints[RIGHT_FRONT].z = 90;

  endpoints[RIGHT_REAR].x = 0;
  endpoints[RIGHT_REAR].y = 134;
  endpoints[RIGHT_REAR].z = 90;

  endpoints[LEFT_FRONT].x = 0;
  endpoints[LEFT_FRONT].y = -134;
  endpoints[LEFT_FRONT].z = 90;

  endpoints[LEFT_REAR].x = 0;
  endpoints[LEFT_REAR].y = -134;
  endpoints[LEFT_REAR].z = 90;

  liftHeight = 30;
  stepsInCycle = 1;
  step = 0;
}

#include "gaits.h"

/* Convert radians to servo position offset. */
int radToServo(float rads) {
  float val = (rads * 100) / 51 * 100;
  return (int) val;
}

/* Body IK solver: compute where legs should be. */
ik_req_t bodyIK(int X, int Y, int Z, int Xdisp, int Ydisp, float Zrot) {
  ik_req_t ans;

  float cosB = cos(bodyRotX);
  float sinB = sin(bodyRotX);
  float cosG = cos(bodyRotY);
  float sinG = sin(bodyRotY);
  float cosA = cos(bodyRotZ + Zrot);
  float sinA = sin(bodyRotZ + Zrot);

  int totalX = X + Xdisp + bodyPosX;
  int totalY = Y + Ydisp + bodyPosY;

  ans.x = totalX - int(totalX * cosG * cosA + totalY * sinB * sinG * cosA + Z * cosB * sinG * cosA - totalY * cosB * sinA + Z * sinB * sinA) + bodyPosX;
  ans.y = totalY - int(totalX * cosG * sinA + totalY * sinB * sinG * sinA + Z * cosB * sinG * sinA + totalY * cosB * cosA - Z * sinB * cosA) + bodyPosY;
  ans.z = Z - int(-totalX * sinG + totalY * sinB * cosG + Z * cosB * cosG);

  return ans;
}

/* Simple 3dof leg solver. X,Y,Z are the length from the Coxa rotate to the endpoint. */
ik_sol_t legIK(int X, int Y, int Z) {
  ik_sol_t ans;

  // first, make this a 2DOF problem... by solving coxa
  ans.coxa = radToServo(atan2(X, Y));
  long trueX = sqrt(sq((long)X) + sq((long)Y)) - L_COXA;
  long im = sqrt(sq((long)trueX) + sq((long)Z));  // length of imaginary leg

  // get femur angle above horizon...
  float q1 = -atan2(Z, trueX);
  long d1 = sq(L_FEMUR) - sq(L_TIBIA) + sq(im);
  long d2 = 2 * L_FEMUR * im;
  float q2 = acos((float)d1 / (float)d2);
  ans.femur = radToServo(q1 + q2);

  // and tibia angle from femur...
  d1 = sq(L_FEMUR) - sq(im) + sq(L_TIBIA);
  d2 = 2 * L_TIBIA * L_FEMUR;
  ans.tibia = radToServo(acos((float)d1 / (float)d2) - 1.57);

  return ans;

}

void doIK() {
  int servo;
  ik_req_t req, gait;
  ik_sol_t sol;

  gaitSetup();

  // right front leg
  gait = gaitGen(RIGHT_FRONT);
  req = bodyIK(endpoints[RIGHT_FRONT].x + gait.x, endpoints[RIGHT_FRONT].y + gait.y, endpoints[RIGHT_FRONT].z + gait.z, X_COXA, Y_COXA, gait.r);
  sol = legIK(endpoints[RIGHT_FRONT].x + req.x + gait.x, endpoints[RIGHT_FRONT].y + req.y + gait.y, endpoints[RIGHT_FRONT].z + req.z + gait.z);

  servo = MID_POSS [COXA_FR] + sol.coxa;
  if (servo < limitMax[COXA_FR] && servo > limitMin[COXA_FR]) {
    bioloid.setNextPose(COXA_FR, servo);
  }
  servo = MID_POSS [FEMUR_FR] - sol.femur;
  if (servo < limitMax[FEMUR_FR] && servo > limitMin[FEMUR_FR]) {
    bioloid.setNextPose(FEMUR_FR, servo);
  }
  servo = MID_POSS [TIBIA_FR] + sol.tibia;
  if (servo < limitMax[TIBIA_FR] && servo > limitMin[TIBIA_FR]) {
    bioloid.setNextPose(TIBIA_FR, servo);
  }

  // right rear leg
  gait = gaitGen(RIGHT_REAR);
  req = bodyIK(endpoints[RIGHT_REAR].x + gait.x, endpoints[RIGHT_REAR].y + gait.y, endpoints[RIGHT_REAR].z + gait.z, -X_COXA, Y_COXA, gait.r);
  sol = legIK(-endpoints[RIGHT_REAR].x - req.x - gait.x, endpoints[RIGHT_REAR].y + req.y + gait.y, endpoints[RIGHT_REAR].z + req.z + gait.z);

  servo = MID_POSS [COXA_BR] - sol.coxa;
  if (servo < limitMax[COXA_BR] && servo > limitMin[COXA_BR]) {
    bioloid.setNextPose(COXA_BR, servo);
  }
  servo = MID_POSS [FEMUR_BR] - sol.femur;
  if (servo < limitMax[FEMUR_BR] && servo > limitMin[FEMUR_BR]) {
    bioloid.setNextPose(FEMUR_BR, servo);
  }
  servo = MID_POSS [TIBIA_BR] + sol.tibia;
  if (servo < limitMax[TIBIA_BR] && servo > limitMin[TIBIA_BR]) {
    bioloid.setNextPose(TIBIA_BR, servo);
  }
  // left front leg
  gait = gaitGen(LEFT_FRONT);
  req = bodyIK(endpoints[LEFT_FRONT].x + gait.x, endpoints[LEFT_FRONT].y + gait.y, endpoints[LEFT_FRONT].z + gait.z, X_COXA, -Y_COXA, gait.r);
  sol = legIK(endpoints[LEFT_FRONT].x + req.x + gait.x, -endpoints[LEFT_FRONT].y - req.y - gait.y, endpoints[LEFT_FRONT].z + req.z + gait.z);

  servo = MID_POSS [COXA_FL] - sol.coxa;
  if (servo < limitMax[COXA_FL] && servo > limitMin[COXA_FL]) {
    bioloid.setNextPose(COXA_FL, servo);
  }
  servo = MID_POSS [FEMUR_FL] - sol.femur;
  if (servo < limitMax[FEMUR_FL] && servo > limitMin[FEMUR_FL]) {
    bioloid.setNextPose(FEMUR_FL, servo);
  }
  servo = MID_POSS [TIBIA_FL] + sol.tibia;
  if (servo < limitMax[TIBIA_FL] && servo > limitMin[TIBIA_FL]) {
    bioloid.setNextPose(TIBIA_FL, servo);
  }

  // left rear leg
  gait = gaitGen(LEFT_REAR);
  req = bodyIK(endpoints[LEFT_REAR].x + gait.x, endpoints[LEFT_REAR].y + gait.y, endpoints[LEFT_REAR].z + gait.z, -X_COXA, -Y_COXA, gait.r);
  sol = legIK(-endpoints[LEFT_REAR].x - req.x - gait.x, -endpoints[LEFT_REAR].y - req.y - gait.y, endpoints[LEFT_REAR].z + req.z + gait.z);

  servo = MID_POSS [COXA_BL] + sol.coxa;
  if (servo < limitMax[COXA_BL] && servo > limitMin[COXA_BL]) {
    bioloid.setNextPose(COXA_BL, servo);
  }
  servo = MID_POSS [FEMUR_BL] - sol.femur;
  if (servo < limitMax[FEMUR_BL] && servo > limitMin[FEMUR_BL]) {
    bioloid.setNextPose(FEMUR_BL, servo);
  }
  servo = MID_POSS [TIBIA_BL] + sol.tibia;
  if (servo < limitMax[TIBIA_BL] && servo > limitMin[TIBIA_BL]) {
    bioloid.setNextPose(TIBIA_BL, servo);
  }

  step = (step + 1) % stepsInCycle;
}
