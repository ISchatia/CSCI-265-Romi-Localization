#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <Rangefinder.h>

#include <Chassis.h>

#define LED_PIN 13

//states for main Sense-Think-Act logic
#define STATE_BEGIN 1
#define STATE_SENSE 2
#define STATE_THINK 3
#define STATE_ACT 4
#define STATE_END 5  

#define CM_TO_INCHES 0.393701

#define MAX_REPS 4

#define WHEEL_DIAMETER 7
#define ENCODER_COUNTS_PER_REV  1440
#define DIST_BETWEEN_WHEELS 14.9

#define MOTOR_K_P 5
#define MOTOR_K_I 0.5

#define SIDE_LENGTH  10
#define TURN_ANGLE  90
#define DRIVE_SPEED 4
#define TURN_SPEED 25 // 15

#define MOTION_DELAY 5

#define IR_DETECTOR_PIN 1

#define ECHO_PIN 11
#define TRIGGER_PIN 4

#define DARK_THRESHOLD 500


#define WANDER_ANGLE 360
#define WANDER_TURN_RATE 15
#define WANDER_SPEED 7
#define WANDER_DIST 10


#define MASK_LINEFOLLOW  0x08
#define MASK_WALLFOLLOW  0x04 
#define MASK_WANDER      0x02
#define MASK_APPROACH    0x01

#define WALL_FOLLOW_ARCH 1
#define WALL_FOLLOW_TURN 2

#define K_P  4.7
#define K_I  0.01
#define K_D  2.3

#define K_P_LINE_FOLLOW 0.3
#define LINE_FOLLOW_SPEED 8 // 5
#define LINE_FOLLOW_THRESHOLD 25

#define LOC_GARAGE 0
#define LOC_GARAGE_DRIVE 1
#define LOC_FLORENCE_SOUTH 2
#define LOC_FLORENCE_NORTH 3
#define LOC_SOUTH_WALK_A 4
#define LOC_SOUTH_WALK_B 5
#define LOC_MAYWOOD_EAST 6
#define LOC_MAIN_WALK 7
#define LOC_MAIN 8
#define LOC_CENTER_WALK 9
#define LOC_WOODLAND 10
#define LOC_HAWTHORNE 11
#define LOC_WOODLAND_HIST_B 12
#define LOC_WOODLAND_HIST_A 13
#define NUM_LOCATIONS 14

#define ACT_FWD 0
#define ACT_LEFT 1
#define ACT_RIGHT 2
#define ACT_NULL 4

#define NUM_ACTIONS 3
#define NUM_ITERATIONS 50000

/*****************************************************/
/*  Glboal Variables                                 */
/*                                                   */
/*  Declare here variables and data structures that  */
/*  remain in memory                                 */
/*****************************************************/

int state= STATE_BEGIN;

int wallFollowState= WALL_FOLLOW_ARCH;

//IR Decoder keypress
int keyPress= 0;

uint8_t behaviorState= 0;


float distance= 0.0;
float inches= 0.0;

float ref= 6;
float meas= 0.0;
float eT= 0.0;
float eT_deltaT= 0.0;
float ddt_eT= 0.0;
float iEt= 0.0;
float actuation= 0.0;


int leftLineSense= 0;
int rightLineSense= 0;
int eLineSense= 0;

bool wanderActivated= false;
bool wallFollowActivated= false;

int testIteration= 0;

bool junctionDetected = false;

int currentLocation = LOC_GARAGE;
bool is_M1 = true; 

int numActions = 0;
int i = 0;
int actions[3];
int selection = 0;
int direction = 0;

//TODO:   global vars for IR decoder, RangeFinder,
//        IR reflectance, and Chassis
Chassis chassis(WHEEL_DIAMETER, ENCODER_COUNTS_PER_REV, DIST_BETWEEN_WHEELS);


Rangefinder rangefinder(ECHO_PIN, TRIGGER_PIN);

bool M1[NUM_LOCATIONS][NUM_ACTIONS][NUM_LOCATIONS] = {
  // LOC_GARAGE 0
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, true , false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_GARAGE_DRIVE 1
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, true , false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_FLORENCE_SOUTH 2
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, true , false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, true , false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_FLORENCE_NORTH 3
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, true , false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_SOUTH_WALK_A 4
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, true , false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_SOUTH_WALK_B 5
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_MAYWOOD_EAST 6
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, true , false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, true , false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_MAIN_WALK 7
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, true , false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_MAIN 8
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, true , false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, true , false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_CENTER_WALK 9
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, false, false, true }, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_WOODLAND 10
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, true }  // ACT_RIGHT 2
  },
  // LOC_HAWTHORNE 11
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, true , false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_WOODLAND_HIST_B 12
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, true , false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_WOODLAND_HIST_A 13
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, true , false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  }
};


bool M2[NUM_LOCATIONS][NUM_ACTIONS][NUM_LOCATIONS] = {
  // LOC_GARAGE 0
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_GARAGE_DRIVE 1
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {true , false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_FLORENCE_SOUTH 2
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, true , false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, true , false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_FLORENCE_NORTH 3
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, true , false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, true }  // ACT_RIGHT 2
  },
  // LOC_SOUTH_WALK_A 4
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, true , false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_SOUTH_WALK_B 5
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, true , false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_MAYWOOD_EAST 6
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, true , false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_MAIN_WALK 7
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, true , false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_MAIN 8
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, true , false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_CENTER_WALK 9
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, true , false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_WOODLAND 10
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, true , false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_HAWTHORNE 11
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, true , false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  },
  // LOC_WOODLAND_HIST_B 12
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, true , false, false}  // ACT_RIGHT 2
  },
  // LOC_WOODLAND_HIST_A 13
  { 
    // 0      1      2      3      4      5      6      7      8      9      10     11     12     13
    {false, false, false, false, false, false, false, false, false, true , false, false, false, false}, // ACT_FWD 0
    {false, false, false, false, false, false, false, false, false, false, true , false, false, false}, // ACT_LEFT 1
    {false, false, false, false, false, false, false, false, false, false, false, false, false, false}  // ACT_RIGHT 2
  }
};

 
uint8_t numActionsLocation1[NUM_LOCATIONS]= {
  1,1,2,1,1,0,2,1,2,1,1,1,1,1
};

uint8_t numActionsLocation2[NUM_LOCATIONS]= {
  0,1,2,2,1,1,1,1,1,1,1,1,1,2
};

uint8_t actionSet1[NUM_LOCATIONS][NUM_ACTIONS]= {
   {ACT_FWD  , ACT_NULL , ACT_NULL}, // LOC_GARAGE 0
   {ACT_LEFT , ACT_NULL , ACT_NULL}, // LOC_GARAGE_DRIVE 1
   {ACT_FWD  , ACT_LEFT , ACT_NULL}, // LOC_FLORENCE_SOUTH 2
   {ACT_FWD  , ACT_NULL , ACT_NULL}, // LOC_FLORENCE_NORTH 3
   {ACT_FWD  , ACT_NULL , ACT_NULL}, // LOC_SOUTH_WALK_A 4
   {ACT_NULL , ACT_NULL , ACT_NULL}, // LOC_SOUTH_WALK_B 5
   {ACT_FWD  , ACT_LEFT , ACT_NULL}, // LOC_MAYWOOD_EAST 6
   {ACT_RIGHT, ACT_NULL , ACT_NULL}, // LOC_MAIN_WALK 7
   {ACT_FWD  , ACT_LEFT , ACT_NULL}, // LOC_MAIN 8
   {ACT_FWD  , ACT_NULL , ACT_NULL}, // LOC_CENTER_WALK 9
   {ACT_RIGHT, ACT_NULL , ACT_NULL}, // LOC_WOODLAND 10
   {ACT_LEFT , ACT_NULL , ACT_NULL}, // LOC_HAWTHORNE 11
   {ACT_RIGHT, ACT_NULL , ACT_NULL}, // LOC_WOODLAND_HIST_B 12
   {ACT_LEFT , ACT_NULL , ACT_NULL}  // LOC_WOODLAND_HIST_A 13
};

uint8_t actionSet2[NUM_LOCATIONS][NUM_ACTIONS]= {
   {ACT_NULL , ACT_NULL , ACT_NULL}, // LOC_GARAGE 0
   {ACT_FWD  , ACT_NULL , ACT_NULL}, // LOC_GARAGE_DRIVE 1
   {ACT_FWD  , ACT_RIGHT, ACT_NULL}, // LOC_FLORENCE_SOUTH 2
   {ACT_LEFT , ACT_RIGHT, ACT_NULL}, // LOC_FLORENCE_NORTH 3
   {ACT_RIGHT, ACT_NULL , ACT_NULL}, // LOC_SOUTH_WALK_A 4
   {ACT_FWD  , ACT_NULL , ACT_NULL}, // LOC_SOUTH_WALK_B 5
   {ACT_FWD  , ACT_NULL , ACT_NULL}, // LOC_MAYWOOD_EAST 6
   {ACT_RIGHT, ACT_NULL , ACT_NULL}, // LOC_MAIN_WALK 7
   {ACT_FWD  , ACT_NULL , ACT_NULL}, // LOC_MAIN 8
   {ACT_LEFT , ACT_NULL , ACT_NULL}, // LOC_CENTER_WALK 9
   {ACT_RIGHT, ACT_NULL , ACT_NULL}, // LOC_WOODLAND 10
   {ACT_FWD  , ACT_NULL , ACT_NULL}, // LOC_HAWTHORNE 11
   {ACT_RIGHT, ACT_NULL , ACT_NULL}, // LOC_WOODLAND_HIST_B 12
   {ACT_FWD  , ACT_LEFT , ACT_NULL}  // LOC_WOODLAND_HIST_A 13
};

/****************************************************/
/*           Behavior and helper routines           */
/****************************************************/


/******
 * Helper routine for debugging hardware
 * while robot running 
 */
void setLED(bool value)
{
  //Serial.println("setLED()");
  digitalWrite(LED_PIN, value);
}


/*****
 *  Stop the motors w/o locking them
 */
void idle() {
   //Serial.println("idle()");
   setLED(HIGH);
 
   //to stop the motors w/o locking wheels
   chassis.idle();
}


/*****
 *   PID Control Law for approch controller
 * 
 */
float controlLawApproach(float Kp, float Ki, float Kd) {
   float result= Kp* eT_deltaT +  Ki* iEt + Kd* ddt_eT;

   return result;
}

/*****
 *   approachController
 * 
 *   Approach obstacle maintain distance actively
 */
void approachController() {
  eT= eT_deltaT;
  eT_deltaT= ref - meas;

  ddt_eT= eT_deltaT - eT;


  /***
   * run if activated
   */
  if ( (behaviorState & MASK_APPROACH) && (fabs(eT_deltaT) > 0.5) ) {

    if (chassis.checkMotionComplete()) {
      //Serial.println("approachController:  checkMotion complete");

      //Serial.println("approachController:  activated");
        /****
         *  If you are far away from the reference
         *  this means you are far away from the goal.
         *  In this case, turn off the Integral component
         *  as it is most useful for steady state error. 
         */
      if (fabs(actuation) > 10) {
        iEt= 0;
      } else {
        iEt+= eT_deltaT;
      }

      //Serial.print("approachController:  meas= ");
      //Serial.print(meas);
      //Serial.print("  eT_deltaT= ");  
      //Serial.print(eT_deltaT);
      //Serial.print("  ddt_eT= ");
      //Serial.print(ddt_eT);
      //Serial.print("  iEt= ");
      //Serial.println(iEt);

      /***
       *  Note:  When robot too close, error is negative
       *         and must drive forward.  When robot is
       *         too far, error is positive and must
       *         drive backwards.  So we negate actuation
       *         signal and use as speed.
       */

      actuation = controlLawApproach(K_P, K_I, K_D);

      //Serial.print("approachController:  actuation= ");
      //Serial.println(actuation);

      /***
       *  If error ( eT_deltaT) is mall and magnitude of actuation is very small, just
       *  call it zero.
       */
      if ( (fabs(actuation) < 0.2) || (fabs(eT_deltaT) < 0.1) ) { 
        actuation= 0.0;
        eT_deltaT= 0.0;
        eT= 0.0;
        chassis.idle();
      } else {

         /*****
         * Note:  In chassis.driveFor(), the way to get it
         *        to move backwards is by using a negative
         *        distance.  It is NOT by using negative
         *        speed which would be more intuitive.
         */
        if (actuation < 0) {
          chassis.driveFor(1, actuation, false);
        } else {
          chassis.driveFor(-1, actuation, false);
        }
      }
    } else {
      delay(MOTION_DELAY);
      //Serial.println("approachController:  checkMotion NOT complete");
    }

  } 
}


/*****
 *   PID Control Law for line Sensor Controller
 * 
 */
float controlLawLineSense(float Kp, float Ki, float Kd) {
   float result= Kp * eLineSense;

   return result;
}


/****** 
 *  line Sense Controller.   Actuates the platform wheels in 
 *  proportion to the error between left and right line sensor.
 */

void lineFollowController() {
  /***
   * run if activated
   */
  if ( behaviorState & MASK_LINEFOLLOW ) {
    //Serial.println("approachController:  activated");

    //if (abs(eLineSense) < LINE_FOLLOW_THRESHOLD) {
    // // eLineSense= 0.0;
    //}

    actuation= controlLawLineSense(K_P_LINE_FOLLOW,0.0,0.0);

    //Serial.print("lineFollowController: actuation= ");
    //Serial.println(actuation);

    chassis.setTwist(LINE_FOLLOW_SPEED,actuation);
  }    
}



/****************************************************/
/*  Sketch Entrypoints here                         */
/****************************************************/

/*****
 *   Power cycle one-shot setup code
 *   Setup board configuration and object/system initializaitons
 */

void setup() {
  // put your setup code here, to run once:

  //srand(1230); // left on 2
  //srand(12356); // foward on 2
  
  // Video Seeds
  //srand(1234); // full loop
  srand(2310); // reverse loop

  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);
  //while(!Serial) {
  //  ;
  //}

  chassis.init();
  chassis.setMotorPIDcoeffs(MOTOR_K_P, MOTOR_K_I);



  rangefinder.init();

  pinMode(LEFT_LINE_SENSE, INPUT);
  pinMode(RIGHT_LINE_SENSE, INPUT);

  state= STATE_BEGIN;

  chassis.driveFor(1,1,true);
  idle();
}


/******
 * Main entrypoint Behavior code
 */

void loop() {
  // put your main code here, to run repeatedly:

  switch(state)  {
    case STATE_BEGIN:
      //Serial.println("STATE_BEGIN");

      behaviorState= 0;
      testIteration= 0;

     
      //Note:  ref is fixed in globals
      // Init the control signals
      meas= 0.0;
      eT= 0.0;
      eT_deltaT= 0.0;
      ddt_eT= 0.0;
      iEt= 0.0;

      wallFollowState= WALL_FOLLOW_ARCH;

      state= STATE_SENSE;
      break;

    case STATE_SENSE:
      //Serial.println();
      //Serial.println("STATE_SENSE");


      //ultrasonic rangefinder
      distance= rangefinder.getDistance();
      //Serial.print("rangefinder distance= ");
      //Serial.println(distance);

      /*****
       * Read the line sensors.    
       * 
       * Big value-  dark (tape detected)
       * Small value- light (no tape detected)
       */
      leftLineSense = analogRead(LEFT_LINE_SENSE);
      rightLineSense = analogRead(RIGHT_LINE_SENSE);
      //Serial.print("leftLineSense= ");
      //Serial.println(leftLineSense);
      //Serial.print("rightLineSense= ");
      //Serial.println(rightLineSense);

      state= STATE_THINK;
      break;

    case STATE_THINK:
      ////Serial.println("STATE_THINK");

      inches= distance * CM_TO_INCHES;
      //Serial.print("rangefinder dist= ");
      //Serial.println(inches);
      meas= inches;

      eLineSense= leftLineSense - rightLineSense;
      // Serial.print("eLineSense= ");
      // Serial.println(eLineSense);

      // Serial.print(eLineSense);
      // Serial.print("   ");
      // Serial.print(leftLineSense);
      // Serial.print("   ");
      // Serial.print(rightLineSense);
      // Serial.println();


      // Junction detection
      if (abs(eLineSense) < 100 && leftLineSense > DARK_THRESHOLD && rightLineSense > DARK_THRESHOLD) {
        junctionDetected =  true;
        chassis.idle();
      }
      else {
        junctionDetected = false;
      }

      /*****
       *  Line following logic
       * 
       *  eLineSense-  line sensor error calcuated by leftValue - rightValue
       * 
       *  1) magnitude less than threshold, either NONE_ON_LINE 
       *  2) eLineSense > threshold, left sensor on line
       *  3) eLineSense < -threshold, right sensor on line
       */

      if (abs(eLineSense) < LINE_FOLLOW_THRESHOLD) {
        //Serial.println("lineSensor:  NONE_ON_LINE");
      } else if (eLineSense > LINE_FOLLOW_THRESHOLD) {
        //Serial.println("lineSensor:  LEFT_ON_LINE");
      } else if (eLineSense < -LINE_FOLLOW_THRESHOLD) {
        //Serial.println("lineSensor:  RIGHT_ON_LINE");
      }
      
      state= STATE_ACT;
      break;

    case STATE_ACT:
      //Serial.println("STATE_ACT");
        
      /****
       *  Test here turning on each
       *  basis behavior.
       * 
       *  Note:  chassis.checkMotionComplete() is called
       *         in each controller because not all controllers
       *         use it.  This makes them self contained.
       */

      //behaviorState|= MASK_APPROACH; 
      //approachController();
      //behaviorState &= (~MASK_APPROACH);

      //behaviorState|= MASK_WANDER;
      //wanderController();
      //behaviorState&= MASK_WANDER;

      // Choose actions based on M1 or M2


      if (junctionDetected) {
        
        Serial.println("JUNCTION DETECTED");

        Serial.print("Current Location: ");
        Serial.println(currentLocation);

        chassis.idle();
        delay(250);

        if (is_M1) {
          numActions = numActionsLocation1[currentLocation];
        }
        else {
          numActions = numActionsLocation2[currentLocation];
        }

        // If actions == 0
        if (numActions == 0) {
          direction = 3;
          is_M1 = !is_M1;
        }
        else {
          // If M1
          if (is_M1) {
            numActions = numActionsLocation1[currentLocation];
            selection = rand() % (numActions);
            for (i = 0; i < 3; i++) {
              actions[i] = actionSet1[currentLocation][i];
            }
            direction = actions[selection];
            for (i = 0; i < NUM_LOCATIONS; i++) {
                if(M1[currentLocation][direction][i]){
                  currentLocation=i;
                  break;
                }
            }
          }
          // If M2
          else {
            numActions = numActionsLocation2[currentLocation];
            selection = rand() % (numActions);
            for (i = 0; i < 3; i++) {
              actions[i] = actionSet2[currentLocation][i];
            }
            direction = actions[selection];
            for (i = 0; i < NUM_LOCATIONS; i++) {
                if(M2[currentLocation][direction][i]){
                  currentLocation=i;
                  break;
                }
            }
          }
        }

        Serial.print("Next Location: ");
        Serial.println(currentLocation);
        
        Serial.print("Selection: ");
        Serial.println(selection);

        Serial.print("Direction: ");
        Serial.println(direction);

        Serial.println();

        // FWD
        if (direction == 0) {
          chassis.driveFor(2, DRIVE_SPEED, true);
        }
        // LEFT
        if (direction == 1) {
          chassis.driveFor(8.25, DRIVE_SPEED, true);
          chassis.turnFor(90, TURN_SPEED, true);
          // chassis.driveFor(-2, DRIVE_SPEED, true);
          // chassis.turnFor(55, TURN_SPEED, true);
        }
        // RIGHT
        if (direction == 2) {
          chassis.driveFor(8.25, DRIVE_SPEED, true);
          chassis.turnFor(-90, TURN_SPEED, true);
          // chassis.driveFor(-2, DRIVE_SPEED, true);
          // chassis.turnFor(-55, TURN_SPEED, true);
        }
        // BACK
        if (direction == 3) {
          chassis.turnFor(180, TURN_SPEED, true);
        }
      }

      behaviorState |= MASK_LINEFOLLOW;
      lineFollowController();
      behaviorState &= (~MASK_LINEFOLLOW);

      testIteration= testIteration + 1;
      //Serial.print("testIteration= ");  
      //Serial.println(testIteration);

      if (testIteration < NUM_ITERATIONS) {
        state = STATE_SENSE;
      } else {
        state= STATE_END;
      }
      break;   

    case STATE_END:
      chassis.idle();
      break;
  }
}
