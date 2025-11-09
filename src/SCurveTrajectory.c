#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "SCurveTrajectory.h"
// #include "can_processing.h"


// From externs in h file
volatile PosCtrlHandle SCurveTrajectory = {0};
volatile VelocityFilter motorTracker    = {0};
volatile bool   newSetpointDetected     = false;
volatile float  positionSetpoint        = 0.0f; // MOVE TO CAN.C

// Private ISR Globals
static PosCtrlHandle  plan_pending;   
static volatile uint32_t sample_count = 0;
static volatile uint8_t replan_request = 0;

// Pointers from planner.c
extern volatile int plan_ready;
extern PosCtrlHandle*    plan_active;

const float armSpeed = 0.08727f; // Basically 5 degree per sec, would be externed by CAN_Processing.c

//just a variable for simulations, nothingmore
[]float pathPlanned;

/*
Returns the maximum angular acceleration based on physical motor parameters
*/
float getAMax(float motorTorque, float motorMomentIntertia) {
    return motorTorque / motorMomentIntertia;
}

/*
Returns the maximum Jerk the arm can output. NOTE: This value is experimentally determined, and is essentially just an indicator
of how long it should take to hit the target angular acceleration to smooth it out instead of it being instantaneous (ms)
*/
float getJMax(float AMax, float timeToAMax) {
    return AMax / (timeToAMax / 1000); // time in ms
}


/*
This function is used to initialize all of the struct parameters initially in the code
*/
void STrajectoryInit(PosCtrlHandle *pHandle, VelocityFilter *motorTracker){

  pHandle->MovementDuration = 0.0f;
  pHandle->AngleStep = 0.0f;
  pHandle->SubStep[0] = 0.0f;
  pHandle->SubStep[1] = 0.0f;
  pHandle->SubStep[2] = 0.0f;
  pHandle->SubStep[3] = 0.0f;
  pHandle->SubStep[4] = 0.0f;
  pHandle->SubStep[5] = 0.0f;

  pHandle->sign = 0;      // -1 back, 0 idle, +1 forward (current), int8_t
  pHandle->sign_prev = 0;  // last tick’s sign, this will be int8_t
  pHandle->remainingDistance = 0; // θ_target - θ

  pHandle->SamplingTime = SAMPLING_TIME; // 0.001f
  pHandle->SubStepDuration = 0;

  pHandle->Jerk = 0.0f;
  pHandle->CruiseSpeed = 0.0f;
  pHandle->Acceleration = 0.0f;
  pHandle->Omega = 0.0f;
  pHandle->OmegaPrev = 0.0f;
  pHandle->Theta = 0.0f;
  pHandle->ThetaPrev = 0.0f;

  pHandle->error_count = 0;
  
  pHandle->A_MAX = getAMax(MOTOR_TORQUE, MOTOR_MOMENT); // M/s^2 - Max angular acceleration
  pHandle-> J_MAX = getJMax(pHandle->A_MAX, 10);
  pHandle-> isExecutingTrajectory = false;
  pHandle->isExecutingBrakingTrajecttory = false;
  velocityFilterInit(motorTracker);
  
}

void velocityFilterInit(VelocityFilter *pHandle){
    pHandle->theta_prev = 0;
    pHandle->omega = 0;
    pHandle->omega_prev = 0;
    pHandle->accel = 0;
    pHandle->alpha_coeff = VEL_FILTER_COEFFICIENT;
    pHandle->Ts = SAMPLING_TIME;
}

/*
This function calculates the required parameters and saves them to the trajectory struct, which 
are used by the algorythm to produce the S curve
  * @param  pHandle handler of the current instance of the Position Control component.
  * @param  startingAngle Current mechanical position.
  * @param  angleStep Target mechanical position.
  * @param  movementDuration Duration to reach the final position (in seconds).

*/
void computeTrajectoryParameters(PosCtrlHandle *pHandle, float startingAngle, float movementDuration, float totalAngleMovement){

    float fMinimumStepDuration;
    fMinimumStepDuration = (9.0f * pHandle->SamplingTime);

    // Round time to fit timesteps of 1kHz
    pHandle->MovementDuration = (float)((int)(movementDuration / fMinimumStepDuration)) * fMinimumStepDuration;

    pHandle->StartingAngle = startingAngle;
    pHandle->AngleStep = totalAngleMovement;
    pHandle->FinalAngle = startingAngle + totalAngleMovement;

    /* SubStep duration = DeltaT/9  (DeltaT represents the total duration of the programmed movement) */
    pHandle->SubStepDuration = (float) pHandle->MovementDuration / 9.0f;

    /* Sub step of acceleration phase */
    pHandle->SubStep[0] = 1 * pHandle->SubStepDuration;   /* Sub-step 1 of acceleration phase */
    pHandle->SubStep[1] = 2 * pHandle->SubStepDuration;   /* Sub-step 2 of acceleration phase */
    pHandle->SubStep[2] = 3 * pHandle->SubStepDuration;   /* Sub-step 3 of acceleration phase */

    /* Sub step of  deceleration Phase */
    pHandle->SubStep[3] = 6 * pHandle->SubStepDuration;   /* Sub-step 1 of deceleration phase */
    pHandle->SubStep[4] = 7 * pHandle->SubStepDuration;   /* Sub-step 2 of deceleration phase */
    pHandle->SubStep[5] = 8 * pHandle->SubStepDuration;   /* Sub-step 3 of deceleration phase */

    /* Jerk (J) to be used by the trajectory calculator to integrate (step by step) the target position.
       J = DeltaTheta/(12 * A * A * A)  => DeltaTheta = final position and A = Sub-Step duration */
    pHandle->Jerk = pHandle->AngleStep / (12 * pHandle->SubStepDuration * pHandle->SubStepDuration * pHandle->SubStepDuration);

    /* Speed cruiser = 2*J*A*A) */
    pHandle->CruiseSpeed = 2 * pHandle->Jerk * pHandle->SubStepDuration * pHandle->SubStepDuration;
    pHandle->ElapseTime = 0.0f;
}

/*
This function is called after each time a new setpoint is provided to the esc, so a 1Khz. It is used to update the velocity
filter object, so that we may extract the best possible state of the current motor being controlled
*/
void updateVelocityFilter(VelocityFilter *pHandle, PosCtrlHandle *pHandletemp){ //Remove the second agument when on ESC as it is a global var
  //This is the version that will work in our escs, but not in testing

  //Apply flitered positions (Filter mostly useful for low speed movements)
  float currentPosition = getCurrentPosition(pHandletemp); // In the read life version, this function takes no parameters, but to simulate we will use extra version, also use rad here
  float omegaRaw = (currentPosition - pHandle->theta_prev)/pHandle->Ts;
  pHandle->omega = pHandle->alpha_coeff * omegaRaw + (1.0f - pHandle->alpha_coeff)* pHandle->omega;
  float accelRaw = (pHandle->omega - pHandle->omega_prev)/pHandle->Ts;
  pHandle->accel = 0.3f * accelRaw + 0.7f * pHandle->accel;

  //update previous positions now
  pHandle->theta_prev = currentPosition;
  pHandle->omega_prev = pHandle->omega;
}

/*
This section is just to select the correct jerk for the right phase in the ramp according to the 9 step procedure
*/
float selectJerk(const PosCtrlHandle *pHandle, float time)
{
  //acceleration
  if (time < pHandle->SubStep[0])      return +pHandle->Jerk;
  else if (time < pHandle->SubStep[1]) return 0.0f;
  else if (time < pHandle->SubStep[2]) return -pHandle->Jerk;

  //constant angular velocity
  else if (time < pHandle->SubStep[2] + 3 * pHandle->SubStep[0]) return 0.0f;

  //decceleration
  else if (time < pHandle->SubStep[3]) return -pHandle->Jerk;
  else if (time < pHandle->SubStep[4]) return 0.0f;
  else if (time < pHandle->SubStep[5]) return +pHandle->Jerk;
  else                return 0.0f;     // done
}

/*
This function is designed to populate the array of positions that will be fed to the ESC in order to properly execue the ramp.

  * @param  planner is the handler of the S curve generator info
  * @param  targetAngle is the desired final position of the joint (rad)
  * @param  angularVelocity is the desired angular velocity of the cruising time (rad/s)
  * @patam  theta_0, omega_0, and accel_0 are the initial conditions in rad, rad/s, and rad/s**2 respectively
*/
void SCurveTrajectoryStarter(PosCtrlHandle *planner, float targetAngle, float angularVelocity){

  // Initialize required parameters
  float t = 0.0f;
  planner->error_count = 0;
  float totalTime = getEstimatedTrajectoryTime(planner->Theta, targetAngle, angularVelocity); // in ms
  float totalAngularMovement = targetAngle - planner->StartingAngle;
  computeTrajectoryParameters(&planner, planner->StartingAngle, totalTime, totalAngularMovement);
  planner->posTol = POS_TOL;
  planner->velTol = VEL_TOL;

  // Activate Status bools
  planner->isExecutingTrajectory = true; 
}

int main (){

  // Put this stuff in the init section
  STrajectoryInit(&SCurveTrajectory, &motorTracker);

  // So what needs to happen is a global var needs to be declared in can_processing.c that is called setpoint, and pretty much anotehr boolean in this file will be 
  // like isNewSetpoint and pretty much it compares the setpoint on each 1khz loop to the current setpoint, and if its different then isNewSetpoint must be set to tru,e which casues 
  // it to reinitioalize the logic and ramp stuff. Otherwise, if a rampp is currently taking place, it just does the math. If it is not, then it would also need to set 
  // the things and run 

}


/*
This function computes the estimated time it will take to execute the desired motion

Inputs: Current joint angle   : rad
        Target Position angel : rad
        desired angular velocity  : rad/sec
Outputs: estimated trajectory time : ms
*/
float getEstimatedTrajectoryTime (float currentAngle, float targetAngle, float angularVelocity){
  return (fabs(currentAngle - targetAngle) / angularVelocity) * 1000 ; // Want ms here
}

/*
This function is to simulate the esc getting an encoder position, for the time being it will simply extract the correct position it should be at at that point in the ramp
*/
float getCurrentPosition(PosCtrlHandle *pHandle){
  return pHandle->Theta;
}

// Local helpter functions -- include the other .c custum fole for these 
float degreesToRad(float positionDegrees){
	return positionDegrees*(M_PI /180.0f);
}

float radToDegrees(float positionRad){
	return (positionRad*180/M_PI);

}

static inline int8_t sign_db(float x, float band) {
    return (x > band) - (x < -band);
}

void PosCtrl_ISRStep(void)
{
    // 0) Acknowledge/pick up newly published plan (planner already swapped pointer)
    if (plan_ready) {
        plan_ready = 0;       // acknowledge
        sample_count = 0;     // restart timing for new plan
    }

    // Alias the active plan once (avoid repeated volatile derefs)
    PosCtrlHandle *P = plan_active;

    // 1) Update fast state (cheap filter); encoder read should be inside updateVelocityFilter()
    //    This keeps omega/accel fresh without heavy math.
    updateVelocityFilter((VelocityFilter*)&motorTracker, P);

    // 2) Light direction/remaining-distance check → request background replan if needed
    const float theta  = P->Theta;                 // planner's current pos (or use motorTracker.theta if you store it)
    const float target = positionSetpoint;         // snapshot once
    const float rem    = target - theta;           // remaining distance
    const int8_t want  = sign_db(rem, P->posTol);  // -1 / 0 / +1 with deadband

    if (want != P->sign) {
        P->sign = want;                // track desired sign for UI/telemetry
        Planner_RequestReplan();       // just sets a flag; heavy work happens in background
    }

    // 3) If no active motion, optionally hold last point in FOLLOW mode and exit
    if (!P->isExecutingTrajectory) {
        // MC_ProgramPositionCommandMotor1(theta, 0.0f); // optional "hold" in follow mode
        return;
    }

    // 4) Jerk select + integrate (strict O(1) hot path)
    const float Ts = (P->SamplingTime > 0.0f) ? P->SamplingTime : SAMPLING_TIME;

    float t = (float)sample_count * Ts;
    if (t > P->MovementDuration) t = P->MovementDuration;

    const float J = selectJerk(P, t);

    // Integrate jerk → accel → vel → pos
    P->Acceleration += J * Ts;

    // Optional physical accel clamp
    if (P->A_MAX > 0.0f) {
        if (P->Acceleration >  P->A_MAX) P->Acceleration =  P->A_MAX;
        if (P->Acceleration < -P->A_MAX) P->Acceleration = -P->A_MAX;
    }

    P->Omega += P->Acceleration * Ts;
    P->Theta += P->Omega        * Ts;

    // 5) Emit next point in FOLLOW mode (duration = 0)
    // MC_ProgramPositionCommandMotor1(P->Theta, 0.0f);

    sample_count++;

    // 6) Completion handling (snap to final; planner can publish next plan if needed)
    if (t >= P->MovementDuration) {
        P->isExecutingTrajectory = false;
        P->Theta = P->FinalAngle;      // ensure exact final position
        // MC_ProgramPositionCommandMotor1(P->Theta, 0.0f);
    }
}
