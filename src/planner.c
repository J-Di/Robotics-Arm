#include <string.h>
#include "planner.h"
#include "SCurveTrajectory.h"
#include <math.h>         

// Variable Declarations
volatile PosCtrlHandle *paths_planned[2];   // path plans from planner.c
volatile uint8_t active_plan;              // active plan by planner.c
volatile uint8_t inactive_plan;            // inactive plan by planner.c
volatile VelocityFilter *motorTracker;
float_t trajTime = 0.0f;                   // overall time of current ramp

//Function prototypes:


/* This function is called once at the start of the code, and it is used to setup the planner for the current ESC. 
    It populates both "active" and "unactive plan", which are used in the 1KHz ISR to control the motor position
*/
void plannerInit(){
 // Start by creating both active and unactive PosCtrlHandle objects
 paths_planned[0] = STrajectoryInit(0.0f); // Start at 0 position
 paths_planned[1] = STrajectoryInit(0.0f); // Start at 0 position
 active_plan = 0; 
 inactive_plan = active_plan | 1;

 // Now initialize the motor tracker object
 motorTracker = velocityFilterInit();


}

/*
This is the main function in planner, that is called once a new setpoint is realized in CAN_Processing Script. 
Depending on the current state of the ongoing plan, this section will determine how to adjust the current plan to 
accomodate the new setpoint. The new plan which is devised will be created in the "inactive_plan" and the flag
"plan_ready" will be set to true, indicating to the ISR that upon the next interrupt to switch over plans to the 
newly prepared one. 
*/
void buildNewCurve(void){

    PosCtrlHandle currPlan = *paths_planned[active_plan];

    // Check to see if we are starting from scratch, or smt is executing
    if (currPlan.isTrajExecuting){
        float_t initialVelocity = calculateVirtualHistory(paths_planned[active_plan], motorTracker);
    }





}

/*
Given a currenlty executing ramp, this function will return what would have been the initial velocity of the ramp,
should it had started with no acceleration. This velocity is essential in order to compute a new ramp, which relies
on the assumption that at the start the motion has no acceleration.
*/
float_t calculateVirtualHistory(PosCtrlHandle *currentMotionPlan, VelocityFilter *motorTracker){

    // start by extracting the initial conditions
    float_t theta0 = motorTracker->theta;
    float_t omega0 = motorTracker->omega;
    float_t 

}


/*
Given the initial conditions of the motor, assuming 0 acceeleration, and the setpoint, this function returns
the critical cutoff times for each of the 7 steps in the S-Curve. This will be used to inform which current phase of 
the ramp the motor is in, and thus the jerk to apply in order to get to the target position. Note if an initial acceleration
is present, then calculateVirtualHistory muust first be called in order to properly simulate the proper condition.
*/
[8]float_t determineSwitchingTimes (float_t targetPos, VelocityFilter *motorTracker){



}

