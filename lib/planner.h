#ifndef PLANNER_H
#define PLANNER_H

#include <stdint.h>
#include <stdbool.h>
#include "SCurveTrajectory.h"

// NOTE: volatile *pointer* (not a pointer to volatile)
extern volatile PosCtrlHandle *  plan_active;
extern volatile uint8_t plan_ready;

void Planner_Init(void);
void Planner_BackgroundTask(void);
void Planner_RequestReplan(void);
void Planner_SetCruiseSpeed(float omega_cruise_rad_s);
void Planner_SetLimits(float a_max_rad_s2, float j_max_rad_s3);

#endif /* PLANNER_H */
