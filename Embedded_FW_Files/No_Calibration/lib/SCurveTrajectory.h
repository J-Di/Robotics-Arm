#ifndef S_CURVE_TRAJECTORY_H
#define S_CURVE_TRAJECTORY_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 
   Motor / trajectory constraint parameters
   TODO: Determine actual values from motor characterization
*/
#define A_MAX       50.0f   // Max acceleration [rad/s^2] og 80
#define J_MAX       10.0f    // Max jerk [rad/s^3] og 20
#define MAX_VEL     1.0f     // Absolute max velocity [rad/s] (~60 deg/s)

/* Sampling and filter parameters */
#define SAMPLING_TIME_PLANNER   0.001f  // 1kHz
#define VEL_FILTER_COEFFICIENT  0.2f    // EMA alpha for velocity filter

/* 
   PosCtrlHandle — stores the full state of one S-curve trajectory plan
*/
typedef struct posCtrlHandle {
    float_t a_max;
    float_t v_max;
    float_t j_max;

    float_t profileSwitchingTimes[8]; // Absolute phase-switch times [t0..t7]
    uint8_t profilePhase;             // Current phase: 1-7 (0 = idle)

    bool isTrajExecuting;  // True while a trajectory is being followed
    bool tooFastPending;    // Indicates if a replan is required for past to fast
    bool wanderReplanPending; // indicates if a replan is required because is wandering
    bool isWandering;      // True if motor must stop then reverse to reach setpoint
    bool isPastTooFast;    // True if virtual v0 > vc during decel-phase target change

    int   dir;             // Motion direction: +1 toward target, -1 away, 0 idle
    float_t theta;         // Planned position [rad] (simulation stand-in for encoder)
} PosCtrlHandle;

/* 
   VelocityFilter — tracks actual motor state from encoder readings
*/
typedef struct {
    float_t theta;          // Current position [rad]
    float_t theta_prev;     // Previous position [rad]
    float_t omega;          // Filtered velocity [rad/s]
    float_t omega_prev;     // Previous velocity [rad/s]
    float_t accel;          // Filtered acceleration [rad/s^2]
    float_t accel_prev;     // Previous acceleration [rad/s^2]
    float_t alpha_coeff;    // EMA filter coefficient (0..1)
    float_t Ts;             // Sample period [s]
    float_t jerk;           // Jerk estimate (mainly for testing)
} VelocityFilter;

/* 
   Externs — variables owned by other modules
*/
extern volatile float positionSetpoint;    // Set by CAN_processing.c
extern volatile bool  newSetpointDetected; // Set by CAN_processing.c

/* 
   Function prototypes
*/

// Initialization
PosCtrlHandle*  STrajectoryInit(float_t currentPos);
VelocityFilter* velocityFilterInit(void);

// Runtime
void    updateVelocityFilter(VelocityFilter *pHandle, PosCtrlHandle *planHandle);
float_t selectJerk(uint8_t profilePhase, float_t jerk);

// Getters
float_t getCurrentPosition(float_t position);  // Simulation stub — returns position as-is
float_t getAngularVelocity(VelocityFilter *pHandle);
float_t getAngularAccel(VelocityFilter *pHandle);
float_t getJerk(VelocityFilter *pHandle);

// Setters
void setMaxVelocity(float_t vel);

// NOTE: degreesToRad / radToDegrees now live in main.h / main.c
// to avoid duplicate symbol errors with the CubeMX-generated project.

#endif /* S_CURVE_TRAJECTORY_H */
