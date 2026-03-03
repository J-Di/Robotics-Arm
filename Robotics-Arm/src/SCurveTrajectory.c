#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "SCurveTrajectory.h"
// #include "can_processing.h"

float_t maxVelocity = 0.02f; // Default value in rad/s

/*
This function is used to initialize and returns a PosCtrlHandle object, which is used to store all the relevent information needed
for a S Curve Trajectory 
*/
PosCtrlHandle* STrajectoryInit(float_t currentPos){

    PosCtrlHandle* pHandle;
    pHandle->a_max = A_MAX;
    pHandle->v_max = maxVelocity;
    pHandle->j_max = J_MAX;
    pHandle->profileSwitchingTimes = {0,0,0,0,0,0,0,0};
    pHandle->profilePhase = 0;
    pHandle->isTrajExecuting = false;
    pHandle->isWandering = false; 
    pHandle->isPastTooFast = false;
    pHandle->theta = currentPos;
    return pHandle;
}

/*
This function initializes and returns a velocity filter object, that will be used to track the motors current status
for the duration of its ramp.
*/
VelocityFilter* velocityFilterInit(){
    VelocityFilter* pHandle;
    pHandle->theta_prev = 0;
    pHandle->omega = 0;
    pHandle->omega_prev = 0;
    pHandle->accel = 0;
    pHandle->alpha_coeff = VEL_FILTER_COEFFICIENT;
    pHandle->Ts = SAMPLING_TIME;
    return pHandle;
}


/*
This function is called after each time a new setpoint is provided to the esc, so a 1Khz. It is used to update the velocity
filter object, so that we may extract the best possible state of the current motor being controlled

NOTE that there are filters available here, but current iteration is without for testing purposes
*/
void updateVelocityFilter(VelocityFilter *pHandle, PosCtrlHandle *PHandleTEMP){ //Remove the second agument when on ESC as it is a global var

  float_t Ts = pHandle->Ts;

  //Apply position Updates --> Some have simple filters which are useful at low speeds
  pHandle->theta_prev = pHandle->theta;
  float_t currentPosition = getCurrentPosition(PHandleTEMP->theta); // In the read life version, this function takes no parameters, but to simulate we will use extra version, also in rad
  pHandle->theta = currentPosition;

  // Now do vel updates too
  pHandle->omega_prev = pHandle->omega;
  float_t currentOmega = (currentPosition - pHandle->theta_prev)/Ts;
  pHandle->omega = currentOmega;
  // pHandle->omega = pHandle->alpha_coeff * currentOmega + (1.0f - pHandle->alpha_coeff)* pHandle->omega_prev;
  // Now do accel updates

  float_t currentAccel = (currentOmega - pHandle->omega_prev)/Ts;
  pHandle->accel = currentAccel;
  // pHandle->accel = 0.3f * currentAccel + 0.7f * pHandle->accel_prev;

}



/*
This section is just to select the correct jerk for the right phase in the ramp according to the current phase
the S trajectory is on. This function is very important as it is what changes the Jerk, which is the only parameter
that we can control to change the overall trajectory.
*/
float_t selectJerk(uint8_t profilePhase, float_t jerk){
switch (profilePhase)
{
  case 1:
    /*Acceleration Phase: +jerk*/
    return jerk;
    break;

  case 2:
    /*Acceleration Phase: no Jerk*/
    return 0;
    break;
  
  case 3:
    /*Acceleration Phase: -Jerk*/
    return -jerk;
    break;

  case 4:
    /*Constant velocity phase: No jerk*/
    return 0;
    break;

  case 5:
    /*Decceleration Phase: -Jerk*/
    return -jerk;
    break;
  
  case 6:
    /*Decceleration Phase: No jerk*/
    return 0;
    break;

  case 7:
    /*Decceleration Phase:  +jerk*/
    return +jerk;
    break;

  default:
    break;
}


}


// C Getter Functions 
/*
This function is to simulate the esc getting an encoder position, for the time being it will simply extract the correct position it should be at at that point in the ramp
*/
float_t getCurrentPosition(float_t position){
  return position;
}

float_t getAngularVelocity(VelocityFilter* pHandle){
    return pHandle->omega;
}

float_t getAngularAccel(VelocityFilter* pHandle){
    return pHandle->accel;
}

float_t getJerk(VelocityFilter* pHandle){
    return pHandle->jerk;
}

// The singular set function
void setMaxVelocity(float_t vel){
  //Perform Checks
  if (abs(vel) > MAX_VEL || abs(vel) < 0.01){ 
    return;
  }
  maxVelocity = vel;
}

// Some Helper Functions --Some only used for simulation
float degreesToRad(float positionDegrees){
	return positionDegrees*(M_PI /180.0f);
}

float radToDegrees(float positionRad){
	return (positionRad*180/M_PI);

}
