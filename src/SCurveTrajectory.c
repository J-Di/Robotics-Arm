#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "SCurveTrajectory.h"
// #include "can_processing.h"


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
void STrajectoryInit(PosCtrlHandle *pHandle){

  pHandle->MovementDuration = 0.0f;
  pHandle->AngleStep = 0.0f;
  pHandle->SubStep[0] = 0.0f;
  pHandle->SubStep[1] = 0.0f;
  pHandle->SubStep[2] = 0.0f;
  pHandle->SubStep[3] = 0.0f;
  pHandle->SubStep[4] = 0.0f;
  pHandle->SubStep[5] = 0.0f;

  pHandle->SamplingTime = SAMPLING_TIME;
  pHandle->SubStepDuration = 0;

  pHandle->Jerk = 0.0f;
  pHandle->CruiseSpeed = 0.0f;
  pHandle->Acceleration = 0.0f;
  pHandle->Omega = 0.0f;
  pHandle->OmegaPrev = 0.0f;
  pHandle->Theta = 0.0f;
  pHandle->ThetaPrev = 0.0f;


  pHandle->A_MAX = getAMax(MOTOR_TORQUE, MOTOR_MOMENT); // M/s^2 - Max angular acceleration
  pHandle-> J_MAX = getJMax(pHandle->A_MAX, 10);
}

void velocityFilterInit(VelocityFilter *pHandle){
    pHandle ->theta_prev = 0;
    pHandle ->omega = 0;
    pHandle ->alpha = 0;
    pHandle ->Ts = SAMPLING_TIME;
}


/*
This function calculates the required parameters and saves them to the trajectory struct, which 
are used by the algorythm to produce the S curve
  * @param  pHandle handler of the current instance of the Position Control component.
  * @param  startingAngle Current mechanical position.
  * @param  angleStep Target mechanical position.
  * @param  movementDuration Duration to reach the final position (in seconds).

*/
float computeTrajectoryParameters(PosCtrlHandle *pHandle, float startingAngle, float movementDuration, float totalAngleMovement){

    float fMinimumStepDuration;
    fMinimumStepDuration = (9.0f * pHandle->SamplingTime);

    /* WARNING: Movement duration value is rounded to the nearest valid value
       [(DeltaT/9) / SamplingTime]:  shall be an integer value */
    pHandle->MovementDuration = (float)((int)(movementDuration / fMinimumStepDuration)) * fMinimumStepDuration;

    pHandle->StartingAngle = startingAngle;
    pHandle->AngleStep = totalAngleMovement;
    pHandle->FinalAngle = startingAngle + totalAngleMovement;

    /* SubStep duration = DeltaT/9  (DeltaT represents the total duration of the programmed movement) */
    pHandle->SubStepDuration = pHandle->MovementDuration / 9.0f;

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
void updateVelocityFilter(VelocityFilter pHandle){
  //This is the version that will work in our escs, but not in testing
  float currentPosition = getCurrentPosition();
  pHandle->omega = currentPosition - pHandle->pre

}


int main (){

  // Put this stuff in the init section
  PosCtrlHandle SCurveTrajectory;
  STrajectoryInit(&SCurveTrajectory);

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

// Local helpter functions -- include the other .c custum fole for these 
float degreesToRad(float positionDegrees){
	return positionDegrees*(M_PI /180.0f);
}

float radToDegrees(float positionRad){
	return (positionRad*180/M_PI);

}