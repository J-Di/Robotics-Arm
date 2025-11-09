#include <stdint.h>
#include <stdbool.h>
#include "SCurveTrajectory.h"   // brings in PosCtrlHandle, VelocityFilter, globals

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


// --- Owned/published by planner.c ---
// The ISR *reads* plan_active (no writing) and checks plan_ready once per tick.
extern PosCtrlHandle *plan_active;   // pointer to the plan the ISR should consume
extern volatile uint8_t plan_ready;  // planner set -> ISR will acknowledge & reset timing

// Initialize the planner’s double buffer and a baseline plan.
void Planner_Init(void);

// Run in your main loop (or a low-priority task).
// Consumes newSetpointDetected and any internal replan requests,
// builds a new plan, and publishes it atomically for the ISR.
void Planner_BackgroundTask(void);

// Lightweight way for the ISR (or others) to request a replan.
// (e.g., when direction sign flips, or you need to truncate to DECEL)
void Planner_RequestReplan(void);

// Optional knobs (call anytime from main loop)
void Planner_SetCruiseSpeed(float omega_cruise_rad_s);
void Planner_SetLimits(float a_max_rad_s2, float j_max_rad_s3);

