#include <string.h>
#include "planner.h"
#include "SCurveTrajectory.h"
#include <math.h>         

// Variable Declarations
volatile PosCtrlHandle paths_planned[2];   // path plans from planner.c
volatile uint8_t active_plan;              // active plan by planner.c
volatile uint8_t inactive_plan;            // inactive plan by planner.c

float_t trajTime = 0.0f;                   // overall time of current ramp

//Function prototypes:

void plannerInit(){
 // continue here to implement the logic!
}

void PosCtrl_ISRStep(void){



}

