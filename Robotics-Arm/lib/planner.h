#ifndef PLANNER_H
#define PLANNER_H

#include <stdint.h>
#include <stdbool.h>
#include "SCurveTrajectory.h"


// Local Vars declared in this file and 
extern volatile PosCtrlHandle *paths_planned[2];    // path plans from planner.c, extern to main.c for interrupt processing
extern volatile VelocityFilter *motorTracker;       // by planner.c, , extern to main.c for interrupt processing
extern volatile uint8_t active_plan;              // active plan by planner.c, , extern to main.c for interrupt processing
extern volatile uint8_t inactive_plan;            // inactive plan by planner.c, , extern to main.c for interrupt processing

// Vars from main.c

// Structs

// Structure that houses virtual history of the current moving motor, mainly for return
struct virtualHistory{
    float_t virt_v0;
    float_t virt_s0;
};

//Structure that houses the esc phase switching times, that will determine which jerk to apply
struct switchingTimes{
    float_t t1;
    float_t t2;
    float_t t3;
    float_t t4;
    float_t t5;
    float_t t6;
    float_t t7;
};

// Function prototypes:
void PosCtrl_ISRStep(void);
#endif /* PLANNER_H */

