#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "SCurveTrajectory.h"


float getAMax(float motorTorque, float motorMomentIntertia) {
    return motorTorque / motorMomentIntertia;
}

float getJMax(float AMax, float timeToAMax) {
    return AMax / timeToAMax;
}

void STrajectoryInit(PosCtrlHandle *pHandle){

  pHandle->MovementDuration = 0.0f;
  pHandle->AngleStep = 0.0f;
  pHandle->SubStep[0] = 0.0f;
  pHandle->SubStep[1] = 0.0f;
  pHandle->SubStep[2] = 0.0f;
  pHandle->SubStep[3] = 0.0f;
  pHandle->SubStep[4] = 0.0f;
  pHandle->SubStep[5] = 0.0f;

  pHandle->SubStepDuration = 0;

  pHandle->Jerk = 0.0f;
  pHandle->CruiseSpeed = 0.0f;
  pHandle->Acceleration = 0.0f;
  pHandle->Omega = 0.0f;
  pHandle->OmegaPrev = 0.0f;
  pHandle->Theta = 0.0f;
  pHandle->ThetaPrev = 0.0f;


  phandle->A_MAX = getAMax(MOTOR_TORQUE, MOTOR_MOMENT); // M/s^2 - Max angular acceleration
  phandle-> J_MAX; // M/s^3 - Max rate of change of angular acceleration
}



int main (){

  // Put this stuff in the init section

  PosCtrlHandle SCurveTrajectory;
  STrajectoryInit(&SCurveTrajectory);
  

}