#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef S_CURVE_TRAJECTORY_H
#define S_CURVE_TRAJECTORY_H

//Motor based parameters -> must calculate
#define A_MAX 100   // FIGURE THIS OUT 
#define J_MAX 20     // FIGURE THIS OUT
#define MAX_VEL 1.4 // FIGURE THIS OUT idk whatever 40 degress/s is in rad

//These are for the elbow motor, and will need to make a helper function to extract for rover depending on esc
#define SAMPLING_TIME 0.001 // in s, 1kHz
#define VEL_FILTER_COEFFICIENT 0.2 // Alter this for smoothening out filter

typedef struct
{
  float_t a_max;
  float_t v_max;
  float_t j_max;

  float_t profileSwitchingTimes[8]; // Array of times where profile state switches [t0,t1,t2,t3,t4,t5,t6,t7]
  uint8_t profilePhase; //Current Phase of the profile: 1,2,3,4,5,6,7 (1-3: Accel) | 4: Constant V | (5-7: Decel), 8, 9 ,10 are special cases
     
  bool isTrajExecuting; // A boolean to indicate if a current profile is currently being executed 
  bool isWandering; // A boolean to indicate if a profile must stop, and then move backwards to hit a certain setpoint
  bool isPastTooFast; // A boolean to indicate if the virtual ramp velocity is faster than the current, in which 

  float_t theta; // TEMPORARY, TO MAKE WORK WITHOUT ENCODER TELLING YOU ANGLE Just feeds where it should have been based om 
                //  the ramp simulated -> ideal case
} PosCtrlHandle;

/*
This struct is to be used as a velocity filter that holds the acual values of the motor state, retrieved and updated through the encoder itself
*/
typedef struct {
    float_t theta;
    float_t theta_prev;     // last angle reading [rad]
    float_t omega;          // filtered velocity [rad/s]
    float_t omega_prev;
    float_t accel;
    float_t accel_prev;
    float_t alpha_coeff;    // filter coefficient (0..1)
    float_t Ts;             // sample period [s]
    float_t jerk;           // This is mostly included for testing purposes
} VelocityFilter;

// Var declared in this file

// Externs Vars
// extern volatile PosCtrlHandle paths_planned[2];   // path plans from planner.c
// extern volatile uint8_t active_plan;              // active plan by planner.c
// extern volatile uint8_t inactive_plan;            // inactive plan by planner.c
// extern volatile VelocityFilter motorTracker;      // motor state tracker from planner.c

extern volatile float positionSetpoint;           // set by CAN_processing.c
extern volatile bool newSetpointDetected;         // set by CAN_processing.c

//Relevent Function Prototypes
float_t getJerk(VelocityFilter* pHandle);
float_t getAngularAccel(VelocityFilter* pHandle);
float_t getAngularVelocity(VelocityFilter* pHandle);
float_t getCurrentPosition(float_t position); // Special Case as might wanna return encoder adn degrees instead of this radians one

//Helpers
float degreesToRad(float positionDegrees);
float radToDegrees(float positionRad);
float getCurrentPosition(float position);

#endif /* S_CURVE_TRAJECTORY_H */