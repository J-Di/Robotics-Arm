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


// Struct prototypes 
static switchingTimes currTrajCutoff;    

/* This function is called once at the start of the code, and it is used to setup the planner for the current ESC. 
    It populates both "active" and "unactive plan", which are used in the 1KHz ISR to control the motor position
*/
int plannerInit(){
 // Start by creating both active and unactive PosCtrlHandle objects
 paths_planned[0] = STrajectoryInit(0.0f); // Start at 0 position
 paths_planned[1] = STrajectoryInit(0.0f); // Start at 0 position

 if (paths_planned[0] || paths_planned[1] == NULL){
    return -1;
 }
 active_plan = 0; 
 inactive_plan = active_plan | 1;

 // Now initialize the motor tracker object
 motorTracker = velocityFilterInit();
 return 0;
}

/*
This is the main function in planner, that is called once a new setpoint is realized in CAN_Processing Script. 
Depending on the current state of the ongoing plan, this section will determine how to adjust the current plan to 
accomodate the new setpoint. The new plan which is devised will be created in the "inactive_plan" and the flag
"plan_ready" will be set to true, indicating to the ISR that upon the next interrupt to switch over plans to the 
newly prepared one. 
*/
void buildNewCurve(float newSetpoint){

    PosCtrlHandle *currPlan = paths_planned[active_plan];

    // local function vars
    float_t newInitialVel;
    float_t newInitialPos;

    // Update initial positions based on if ramp is being executed or not, to be fed into sectionA
    if (currPlan->isTrajExecuting){
        // Section B is invoked
        virtualHistory* virtualInitialCond = calculateVirtualHistory(paths_planned[active_plan], motorTracker);
        newInitialVel = virtualInitialCond->virt_v0;
        newInitialPos = virtualInitialCond->virt_s0;
    }
    else{
        newInitialVel = motorTracker->omega;
        newInitialPos = motorTracker->theta;
    }

    //Now create new ramp based on initial parameters
    if (currPlan->isPastTooFast){ // if too fast, must recalculate on next iteration, forced -J_MAX
        return;
    }

    calculateNewRamp(paths_planned[inactive_plan], motorTracker, newInitialPos, newInitialVel, newSetpoint);


}

/*
Given a currenlty executing ramp, this function will return what would have been the initial velocity of the ramp,
should it had started with no acceleration. This velocity is essential in order to compute a new ramp, which relies
on the assumption that at the start the motion has no acceleration.
*/
virtualHistory* calculateVirtualHistory(PosCtrlHandle *currentMotionPlan, VelocityFilter *motorTracker){

    // start by extracting the initial conditions
    float_t sc = motorTracker->theta;
    float_t vc = motorTracker->omega;
    float_t ac = motorTracker->accel;

    // maximum values
    float_t jmax = currentMotionPlan->j_max;
    float_t amax = currentMotionPlan->a_max;

    // virtual initial parameters
    float_t virt_v0;
    float_t virt_s0; 

    // calculate virtual history of current motor status, refer to flowchart for better understanding
    switch(currentMotionPlan->profilePhase){

        case (1):
            virt_v0 =   vc - (ac/2) * (ac/jmax);
            virt_s0 =   sc - (ac/6) * pow((ac/jmax), 2.0);
            break;
        case(2):
            virt_v0 =   vc - (amax/2) * (amax/jmax);
            virt_s0 =   sc - (amax/6) * pow((amax/jmax), 2.0);
            break;
        case(3):
            virt_v0 =   vc - (ac/2) * (ac/jmax);
            virt_s0 =   sc - (ac/6) * pow((ac/jmax), 2.0);
            break;
        case(4):
            // This is just the baseline condition
            virt_v0 = vc;
            virt_s0 = sc;
            break;
        case(5):
            currentMotionPlan->isPastTooFast = false;
            virt_v0 =   vc - (ac/2) * (ac/jmax); 
            if (virt_v0 > vc ){
                // we are in the case where the virtual velocity is greater than current
                currentMotionPlan->isPastTooFast = true;
                virt_s0 = 0.00f; // arbitrary value for now, wont effect result
            }
            else{
                virt_s0 = sc - (ac/6) * pow((ac/jmax), 2.0) + virt_v0 * (ac/jmax);
            }
            break;
        case(6):
            currentMotionPlan->isPastTooFast = false;
            virt_v0 =   vc + (amax/2) * (ac/jmax); 
            if (virt_v0 > vc ){
                // we are in the case where the virtual velocity is greater than current
                currentMotionPlan->isPastTooFast = true;
                virt_s0 = 0.00f; // arbitrary value for now, wont effect result
            }
            else{
                virt_s0 = sc + (amax/6) * pow((amax/jmax), 2.0) - virt_v0 * (ac/jmax);
            }
            break;
        case(7):
            currentMotionPlan->isPastTooFast = false;
            virt_v0 =   vc - (ac/2) * (ac/jmax); 
            if (virt_v0 > vc ){
                // we are in the case where the virtual velocity is greater than current
                currentMotionPlan->isPastTooFast = true;
                virt_s0 = 0.00f; // arbitrary value for now, wont effect result
            }
            else{
                virt_s0 = sc - (ac/6) * pow((ac/jmax), 2.0) + virt_v0 * (ac/jmax);
            }
            break;
        
        default:
            break;
    }

    // create virtual history object
    virtualHistory historyParameters = {virt_v0, virt_s0};

    return &historyParameters;
}


/*
Given the initial conditions of the motor, assuming 0 acceeleration, and the setpoint, this function returns
the critical cutoff times for each of the 7 steps in the S-Curve. This will be used to inform which current phase of 
the ramp the motor is in, and thus the jerk to apply in order to get to the target position. Note if an initial acceleration
is present, then calculateVirtualHistory muust first be called in order to properly simulate the proper condition.
*/
switchingTimes determineSwitchingTimes (float_t targetPos, float_t s0, float_t v0, PosCtrlHandle *currentMotionPlan, VelocityFilter *motorTracker){

    float_t a_max = currentMotionPlan->a_max;   
    float_t j_max = currentMotionPlan->j_max;  

    float_t dir  = (targetPos - s0 >= 0) ? 1.0f : -1.0f;
    float_t v0n  = dir * v0;                    // signed in toward target is +
    float_t sEn  = dir * (targetPos - s0);      // >= 0

    // If we're moving away from the target, brake to 0 first, then replan toward the target.
    if (v0n <= 0.0f) {
        // handle "moving away" branch (turnaround / stop then replan)
        currTrajCutoff.isWandering = true;
        return decellerate2Stop(targetPos, s0, v0, currentMotionPlan, motorTracker);
    }

    // Case e: minimum stopping distance with jerk/acc limits (using v0n >= 0)
    float_t v = v0n;
    float_t v_thresh = (a_max * a_max) / j_max; 

    float_t deltaSmin;
    if (v >= v_thresh) {  // e1: reaches -a_max
        deltaSmin = (v * 0.5f) * (v / a_max + a_max / j_max);
    } else {              // e2: jerk-limited only, doesn't reach -a_max
        deltaSmin = v * sqrtf(v / j_max);
    }

    if (sEn < deltaSmin) {
        // handle "moving away" branch (turnaround / stop then replan)
        currTrajCutoff.isWandering = true;
        return decellerate2Stop(targetPos, s0, v0, currentMotionPlan, motorTracker);
    } 


}

/*
This function is used to decellerate to a stop. It can be called either to stop the motor, or if the motor needs to switch directions 
to get to a setpoint. This happens if the next setpoint is in the opposite direction, or if the motor cannot slow down in time to get to 
the desired target, and must stop and reprogram.  
*/
switchingTimes decellerate2Stop (float_t targetPos, float_t s0, float_t v0, PosCtrlHandle *currentMotionPlan, VelocityFilter *motorTracker){

    // Extract parameters
    float_t a_max = currentMotionPlan->a_max;   
    float_t j_max = currentMotionPlan->j_max;  

    // First we need to bring the motor to a stop, regardless of where the current setpoint is 
    // float_t dirBrake = (v0 >= 0.0f) ? 1.0f : -1.0f;/
    float_t v = fabsf(v0);
    float_t dt5 = 0.0f;
    float_t dt6 = 0.0f;
    float_t dt7 = 0.0f;

    // Now determine the braking profile based on cases discussed in e 
    float_t v_thresh = (a_max * a_max) / j_max;
    if (v >= v_thresh) {
        // Case e1: reaches max accel
        float_t dt5 = a_max / j_max;
        float_t dt6 = v / a_max - a_max / j_max;
        float_t dt7 = a_max / j_max;
    } else {
        // Case e2: does not reach max accel
        float_t dt5 = sqrtf(v / j_max);
        float_t dt6 = 0.0f;
        float_t dt7 = dt5;
    }

    // Now populate the trajectory cutoff times struct 
    currTrajCutoff.t0 = 0.0f;
    currTrajCutoff.t1 = 0.0f;
    currTrajCutoff.t2 = 0.0f;
    currTrajCutoff.t3 = 0.0f;
    currTrajCutoff.t4 = 0.0f;
    currTrajCutoff.t5 = dt5;
    currTrajCutoff.t6 = dt5 + dt6;
    currTrajCutoff.t7 = dt5+dt6+dt7;
    return currTrajCutoff;
}