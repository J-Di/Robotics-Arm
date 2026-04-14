#ifndef VELOCITY_CTRL_H
#define VELOCITY_CTRL_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "SCurveTrajectory.h"

/*
   VelCtrlHandle — state for jerk-limited velocity tracking mode.

   This module provides joystick / CAN velocity control that outputs a
   position setpoint compatible with the existing PID position loop.

   The demanded velocity arrives over CAN.  Each ISR tick the filter
   applies jerk-limited, acceleration-limited ramping to track the
   demand and integrates the resulting velocity into a position setpoint.

   The output is written to the same VelocityFilter tracker that the
   S-curve planner uses, so the rest of the system (PID, encoder
   feedback) doesn't need to know which mode is active.
*/

typedef struct {
    /* Constraint limits (copied from system constants at init) */
    float_t a_max;
    float_t v_max;
    float_t j_max;
    float_t Ts;

    /* Internal filter state */
    float_t cmd_vel;       // Current commanded velocity [rad/s] (output of the filter)
    float_t cmd_accel;     // Current commanded acceleration [rad/s^2]
    float_t cmd_pos;       // Accumulated position setpoint [rad]

    /* Demand input */
    float_t demanded_vel;  // Target velocity from CAN [rad/s]

    /* Position limits (optional, 0 = disabled) */
    float_t pos_min;       // Minimum allowed position [rad]
    float_t pos_max;       // Maximum allowed position [rad]
    bool    pos_limits_en; // True if position limits are active

    /* Status */
    bool    is_active;     // True when velocity control mode is running
} VelCtrlHandle;

/*
   Function prototypes
*/

/* Static initialization for bare-metal STM32 (no malloc).
   Caller provides a pre-allocated VelCtrlHandle.
   currentPos: the motor's current position (sets the initial accumulated position). */
void velCtrlInitStatic(VelCtrlHandle *h, float_t currentPos);

/* Set the demanded velocity.  Called from CAN processing or main loop
   when a new velocity command arrives.  The value is clamped to +/- v_max. */
void velCtrlSetDemand(VelCtrlHandle *h, float_t demandedVel);

/* Enable or disable software position limits.
   When enabled, the filter will decelerate to a stop before hitting the limit. */
void velCtrlSetPositionLimits(VelCtrlHandle *h, float_t minPos, float_t maxPos);
void velCtrlClearPositionLimits(VelCtrlHandle *h);

/* Activate velocity control mode.  Initialises the filter state from
   the motor tracker so the transition is seamless. */
void velCtrlStart(VelCtrlHandle *h, VelocityFilter *tracker);

/* Deactivate velocity control mode.  The filter decelerates to a stop
   on the next ticks (caller should keep calling the ISR step until
   cmd_vel reaches zero, then switch to position-hold or S-curve mode). */
void velCtrlStop(VelCtrlHandle *h);

/* ISR tick — call at 1 kHz.
   Computes one step of the jerk-limited velocity filter:
     1. Decide jerk direction to track demanded velocity
     2. Clamp acceleration to a_max
     3. Integrate jerk → accel → velocity → position
     4. Write state to the tracker
   Returns the new position setpoint (also stored in h->cmd_pos). */
float_t velCtrl_ISRStep(VelCtrlHandle *h, VelocityFilter *tracker);

/* Read current filter state (for telemetry / logging) */
float_t velCtrlGetVelocity(VelCtrlHandle *h);
float_t velCtrlGetAccel(VelCtrlHandle *h);
float_t velCtrlGetPosition(VelCtrlHandle *h);

#endif /* VELOCITY_CTRL_H */
