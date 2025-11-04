#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "SCurveTrajectory.h"
// #include "can_processing.h"

//Declarations
volatile PosCtrlHandle SCurveTrajectory;
volatile VelocityFilter motorTracker;
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

    /* WARNING: Movement duration value is rounded to the nearest valid value
       [(DeltaT/9) / SamplingTime]:  shall be an integer value */
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

  // Activate Status bools
  planner->isExecutingTrajectory = true; 
}

/* This is the function that is called at 1kHz by the microcontroller to update the escs position according to the algorithm, in the ISR. IT takes the 
    S curve planner and the veolicty filter as inputs, and uses it to calculate the enxt ange according to the algorithm, and writes it to the ESC.
    It Must be a quick one not to take up too much room in the ISR.
 */
void PosCtrl_ISRStep(void){

    const float Ts = SCurveTrajectory.SamplingTime;

    // Capture current state of the motor
    updateVelocityFilter(&motorTracker, &SCurveTrajectory);
    float theta0 = motorTracker.theta_prev; // This is actually the current angle, defined this way for the integration
    float omega = motorTracker.omega;

    // Update remaining distance and the sign
    SCurveTrajectory.remainingDistance = positionSetpoint - theta0;
    int new_sign = sign_with_deadband(SCurveTrajectory.remainingDistance, POS_TOL);

    // Check sign difference
    if (new_sign != SCurveTrajectory.sign){
      SCurveTrajectory.sign_prev = SCurveTrajectory.sign;
      SCurveTrajectory.sign = new_sign;

      //Now Check to see what the new sign is
      if (new_sign == 0){
        // We're done with the ramp
        SCurveTrajectory.isExecutingTrajectory = false;
      }
      else{
        //Direction changed
        if (omega * (float)new_sign < -VEL_TOL){
          // If we are moving opposite to the velocity, we must first plan to stop
          begin_brake_to_zero(&SCurveTrajectory);
        }
        else{
          // Moving in same direciton, replan from where we are right now
          SCurveTrajectory.Theta        = theta0;
          SCurveTrajectory.Omega        = omega;
          SCurveTrajectory.Acceleration = motorTracker.accel;
          SCurveTrajectoryStarter(&SCurveTrajectory, positionSetpoint, armSpeed);
          sample_count = 0;
          SCurveTrajectory.isExecutingTrajectory = true;
        }
      }
    }

    float t = Ts * sample_count;
    if (t > SCurveTrajectory.MovementDuration) t = SCurveTrajectory.MovementDuration;

    //Select Jerk for this portion of the segment based on t
    float J = selectJerk(&SCurveTrajectory, t);

    // integrate
    SCurveTrajectory.Acceleration += J * Ts;
    SCurveTrajectory.Omega        += SCurveTrajectory.Acceleration * Ts;
    SCurveTrajectory.Theta        += SCurveTrajectory.Omega * Ts;

    // send follow point (duration = 0)
    MC_ProgramPositionCommandMotor1(SCurveTrajectory.Theta, 0.0f);

    sample_count++;
    
    if (t >= SCurveTrajectory.MovementDuration) {
        // If this was a BRAKE, you might set a flag and immediately begin_new_profile()
        // toward the (possibly new) positionSetpoint.
        SCurveTrajectory.isExecutingTrajectory = false;
        SCurveTrajectory.Theta = SCurveTrajectory.FinalAngle; // snap final, if applicable
    }

}

void MC_ProgramPositionCommandMotor1(float position, float time){
  pathPlanned
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

static inline int sign_with_deadband(float x, float deadband) //change to int8_t
{
    if (x >  deadband) return +1;
    if (x < -deadband) return -1;
    return 0;
}

static inline void begin_brake_to_zero(PosCtrlHandle *p)
{
    p->isExecutingTrajectory = true;  // still executing, but as a brake
}

