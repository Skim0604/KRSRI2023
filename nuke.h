/******************************************************************************
 * Inverse Kinematics for 4/6 legged bots using 3DOF lizard legs
 *
 * Auto-Generated by NUKE!
 *   http://arbotix.googlecode.com
 *
 * FRONT VIEW       ^        ==0         0==
 *     /\___/\      |       |  0==[___]==0  |
 *    /       \     Z       |               |
 *
 * TOP VIEW
 *    \       /     ^
 *     \_____/      |
 *  ___|     |___   X
 *     |_____|
 *     /     \      Y->
 *    /       \
 *****************************************************************************/

#ifndef NUKE
#define NUKE
                  
#define LEG_COUNT   4

/* Body
 * We assume 4 legs are on the corners of a box defined by X_COXA x Y_COXA
 */
#define X_COXA      47  // MM between front and back legs /2
#define Y_COXA      47  // MM between front/back legs /2

/* Legs */
#define L_COXA      34  // MM distance from coxa servo to femur servo
#define L_FEMUR     45 // MM distance from femur servo to tibia servo
#define L_TIBIA     70 // MM distance from tibia servo to foot

//=======================IDs LEG======================
#define COXA_FR    1
#define COXA_BR    2
#define COXA_FL    3
#define COXA_BL    4

#define FEMUR_FR   5
#define FEMUR_BR   6
#define FEMUR_FL   7
#define FEMUR_BL   8

#define TIBIA_FR   9
#define TIBIA_BR   10
#define TIBIA_FL   11
#define TIBIA_BL   12

/* A leg position request (output of body calcs, input to simple 3dof solver). */
typedef struct{
    int x;
    int y;
    int z;
    float r;
} ik_req_t;

/* Servo ouptut values (output of 3dof leg solver). */
typedef struct{
    int coxa;
    int femur;
    int tibia;
} ik_sol_t;

/* Actual positions, and indices of array. */
extern ik_req_t endpoints[LEG_COUNT];
#define RIGHT_FRONT    0
#define RIGHT_REAR     1
#define LEFT_FRONT     2
#define LEFT_REAR      3

extern BioloidController bioloid;

/* Parameters for manipulating body position */
extern float bodyRotX;    // body roll
extern float bodyRotY;    // body pitch
extern float bodyRotZ;    // body rotation
extern int bodyPosX;
extern int bodyPosY;

/* Parameters for gait manipulation */
extern int Xspeed;
extern int Yspeed;
extern float Rspeed;
extern int tranTime;
extern float cycleTime;
extern int stepsInCycle;
extern int liftHeight;
extern int step;
extern int currentGait;

/* Gait Engine */
extern int gaitLegNo[];   // order to move legs in
extern ik_req_t gaits[];  // gait position

/* convert radians to a dynamixel servo offset */
int radToServo(float rads);
/* select a gait pattern to use */
void gaitSelect(int GaitType);

/* find the translation of the endpoint (x,y,z) given our gait parameters */
extern ik_req_t (*gaitGen)(int leg);
extern void (*gaitSetup)();
/* ripple gaits move one leg at a time TODO: THIS SHOULD BE IN NUKE.H
 *  for going forward, or turning left/right
 */
#define RIPPLE                  0
#define RIPPLE_LEFT             1
#define RIPPLE_RIGHT            2
/* amble gaits move two alternate legs at a time */
#define AMBLE                   5

/* find the translation of the coxa point (x,y) in 3-space, given our rotations */
ik_req_t bodyIK(int X, int Y, int Z, int Xdisp, int Ydisp, float Zrot);
/* given our leg offset (x,y,z) from the coxa point, calculate servo values */
ik_sol_t legIK(int X, int Y, int Z);
/* ties all of the above together */
void doIK();
/* setup the starting positions of the legs. */
void setupIK();

#endif