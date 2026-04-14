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
extern volatile bool plan_ready;                  // flag for ISR to swap plans

// Vars from main.c

// Structs

// Structure that houses virtual history of the current moving motor, mainly for return
typedef struct virtualHistory{
    float_t virt_v0;
    float_t virt_s0;
} virtualHistory;

//Structure that houses the esc phase switching times, that will determine which jerk to apply
typedef struct switchingTimes{
    float_t t0;
    float_t t1;
    float_t t2;
    float_t t3;
    float_t t4;
    float_t t5;
    float_t t6;
    float_t t7;
    bool isWandering;
} switchingTimes;

// Function prototypes:
int plannerInit(void);
void buildNewCurve(float newSetpoint);
virtualHistory calculateVirtualHistory(PosCtrlHandle *currentMotionPlan, VelocityFilter *motorTracker);
switchingTimes determineSwitchingTimes(float_t targetPos, float_t s0, float_t v0, PosCtrlHandle *currentMotionPlan, VelocityFilter *motorTracker);
switchingTimes decellerate2Stop(float_t targetPos, float_t s0, float_t v0, PosCtrlHandle *currentMotionPlan, VelocityFilter *motorTracker);
void calculateNewRamp(PosCtrlHandle *newPlan, VelocityFilter *motorTracker, float_t s0, float_t v0, float_t targetPos);
void PosCtrl_ISRStep(void);
void Planner_MainStep(volatile bool *newCommandPending, float requestedSetpoint);

#endif
