#ifndef PLANNER_H
#define PLANNER_H

#include <stdint.h>
#include <stdbool.h>
#include "SCurveTrajectory.h"


// Vars declared in this file
extern volatile PosCtrlHandle paths_planned[2];   // path plans from planner.c
extern volatile uint8_t active_plan;              // active plan by planner.c
extern volatile uint8_t inactive_plan;            // inactive plan by planner.c

//Externs coming from other files


// Function prototypes:

void PosCtrl_ISRStep(void);
#endif /* PLANNER_H */

