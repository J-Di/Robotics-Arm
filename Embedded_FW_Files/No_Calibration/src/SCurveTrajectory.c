#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "main.h"
#include "SCurveTrajectory.h"
// #include "can_processing.h"

/* Runtime-adjustable velocity limit (default very low for safety) */
static float_t maxVelocity = 0.02f;  // [rad/s]

/*
   STrajectoryInit
   Allocates and initializes a PosCtrlHandle for one trajectory plan buffer.
   Returns NULL on allocation failure.
*/
PosCtrlHandle* STrajectoryInit(float_t currentPos){

    PosCtrlHandle *pHandle = malloc(sizeof(PosCtrlHandle));
    if (!pHandle){
        return NULL;
    }

    pHandle->a_max = A_MAX;
    pHandle->v_max = maxVelocity;
    pHandle->j_max = J_MAX;

    for (int i = 0; i < 8; i++){
        pHandle->profileSwitchingTimes[i] = 0.0f;
    }

    pHandle->profilePhase   = 0;
    pHandle->isTrajExecuting = false;
    pHandle->tooFastPending = false;
    pHandle->isWandering    = false;
    pHandle->isPastTooFast  = false;
    pHandle->wanderReplanPending = false;
    pHandle->dir            = 0;     // idle
    pHandle->theta          = currentPos;

    return pHandle;
}
/*
   velocityFilterInit
   Allocates and zero-initializes a VelocityFilter for motor state tracking.
   Returns NULL on allocation failure.
*/
VelocityFilter* velocityFilterInit(void){

    VelocityFilter *pHandle = malloc(sizeof(VelocityFilter));
    if (!pHandle){
        return NULL;
    }

    pHandle->theta       = 0.0f;
    pHandle->theta_prev  = 0.0f;
    pHandle->omega       = 0.0f;
    pHandle->omega_prev  = 0.0f;
    pHandle->accel       = 0.0f;
    pHandle->accel_prev  = 0.0f;
    pHandle->alpha_coeff = VEL_FILTER_COEFFICIENT;
    pHandle->Ts          = SAMPLING_TIME_PLANNER;
    pHandle->jerk        = 0.0f;

    return pHandle;
}



/*
   updateVelocityFilter
   Called each ISR tick to update the velocity filter with the latest motor
   state. In simulation, reads position from planHandle->theta. On real
   hardware, this should read the encoder instead.

   Currently uses raw finite-difference derivatives. EMA filter lines are
   commented out — enable them once real encoder noise is characterized.
*/
void updateVelocityFilter(VelocityFilter *pHandle, PosCtrlHandle *planHandle){

    float_t Ts = pHandle->Ts;

    //  Position update
    pHandle->theta_prev = pHandle->theta;
    // In real firmware: replace with encoder read (no parameter needed)
    float_t currentPosition = Read_Encoder_Position_Rad(); // getCurrentPosition(planHandle->theta); // for sims
    pHandle->theta = currentPosition;

    //  Velocity update
    pHandle->omega_prev = pHandle->omega;
    float_t rawOmega = (currentPosition - pHandle->theta_prev) / Ts;
    pHandle->omega = rawOmega;
    // Filtered version (uncomment when testing with real encoder noise):
    // pHandle->omega = pHandle->alpha_coeff * rawOmega
    //                + (1.0f - pHandle->alpha_coeff) * pHandle->omega_prev;

    //  Acceleration update
    pHandle->accel_prev = pHandle->accel;
    float_t rawAccel = (pHandle->omega - pHandle->omega_prev) / Ts;
    pHandle->accel = rawAccel;
    // Filtered version:
    // pHandle->accel = 0.3f * rawAccel + 0.7f * pHandle->accel_prev;

    //  Jerk update (for diagnostics / testing)
    pHandle->jerk = (pHandle->accel - pHandle->accel_prev) / Ts;
}



/*
This section is just to select the correct jerk for the right phase in the ramp according to the current phase
the S trajectory is on. This function is very important as it is what changes the Jerk, which is the only parameter
that we can control to change the overall trajectory.

  Maps the current S-curve phase (1–7) to the correct jerk sign.
  Paper eq (5):
    Phase 1,7:  +j_max   (jerk ramp up / ramp back)
    Phase 2,4,6: 0       (constant accel or constant vel)
    Phase 3,5:  -j_max   (jerk ramp down)
  Returns 0.0 for any unexpected phase value (including 0 = idle).
*/
float_t selectJerk(uint8_t profilePhase, float_t jerk){

    switch (profilePhase){
        case 1:  return  jerk;   // Phase I:   accel increasing
        case 2:  return  0.0f;   // Phase II:  constant max accel
        case 3:  return -jerk;   // Phase III: accel decreasing
        case 4:  return  0.0f;   // Phase IV:  constant velocity
        case 5:  return -jerk;   // Phase V:   decel increasing
        case 6:  return  0.0f;   // Phase VI:  constant max decel
        case 7:  return  jerk;   // Phase VII: decel decreasing
        default: return  0.0f;   // Idle or invalid phase
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

/*
   setMaxVelocity
   Runtime setter for the velocity limit. Rejects values outside [0.01, MAX_VEL].
   Note: only affects plans created AFTER this call (existing plans keep their
   v_max until re-initialized).
   */
void setMaxVelocity(float_t vel){
    if (fabsf(vel) > MAX_VEL || fabsf(vel) < 0.01f){
        return;
    }
    maxVelocity = vel;
}
