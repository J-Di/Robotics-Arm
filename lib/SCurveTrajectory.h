#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

//These are for the elbow motor, and will need to make a helper function to extract for rover depending on esc
#define MOTOR_TORQUE  405e-3f  // Nm
#define MOTOR_MOMENT 13.31e-6f // kg·m²
#define SAMPLING_TIME 0.001 // in s, 1kHz
#define DEBUG_MODE 1 // 1 for debug, zero for actual motor application
#define VEL_FILTER_COEFFICIENT 0.2 // Alter this for smoothening out filter
#define TRAJ_MAX_POINTS 8000 // NMaximum size of array calculated based on worst case Delta_T/Ts
#define ERROR_TOLERANCE 5 // rad verison of 5 degrees for error tolerance
#define POS_TOL 0.002 // arnd 0.1 degrees
#define VEL_TOL 0.02 //rad /s

typedef struct
{
  float MovementDuration;              /**< @brief Total duration of the programmed movement */
  float StartingAngle;                 /**< @brief Current mechanical position */
  float FinalAngle;                    /**< @brief Target mechanical position including start position */
  float AngleStep;                     /**< @brief Target mechanical position */
  float SubStep[6];                    /**< @brief Sub step interval time of acceleration and deceleration phases */
  float SubStepDuration;               /**< @brief Sub step time duration of sequence : acceleration / cruise /
                                                   deceleration */
  
  int sign;        // -1 back, 0 idle, +1 forward (current), int8_t
  int sign_prev;   // last tick’s sign, this will be int8_t
  float  remainingDistance; // θ_target - θ
  float  posTol;      // deadband on distance (e.g., 0.002 rad ~0.1°)
  float  velTol;      // deadband on speed    (e.g., 0.02 rad/s)
  float ElapseTime;                    /**< @brief Elapse time during trajectory movement execution */
  int SamplingTime;                  /**< @brief Sampling time at which the movement regulation is called
                                                   (at 1/MEDIUM_FREQUENCY_TASK_RATE) */
  float Jerk;                          /**< @brief Angular jerk, rate of change of the angular acceleration with respect
                                                   to time */
  float CruiseSpeed;                   /**< @brief Angular velocity during the time interval after acceleration and
                                                   before deceleration */
  float Acceleration;                  /**< @brief Angular acceleration in rad/s^2 */
  float Omega;                         /**< @brief Estimated angular speed in rad/s */
  float OmegaPrev;                     /**< @brief Previous estimated angular speed of frame (N-1) */
  float Theta;                         /**< @brief Current angular position */
  float ThetaPrev;                     /**< @brief Angular position of frame (N-1) */

  float A_MAX; // M/s^2 - Max angular acceleration
  float J_MAX; // M/s^3 - Max rate of change of angular acceleration
  bool isExecutingTrajectory; // Is the current ESC current executing a movement
  bool isExecutingBrakingTrajecttory; // Is the ESC currently attemping to break
  int error_count; //number that tracks the amount of times the escs current and targetted position are too different
} PosCtrlHandle;

/*
This struct is to be used as a velocity filter that holds the acual values of the motor state, retrieved and updated through the encoder itself
*/
typedef struct {
    float theta_prev;     // last angle reading [rad]
    float omega;          // filtered velocity [rad/s]
    float omega_prev;
    float accel;
    float alpha_coeff;    // filter coefficient (0..1)
    float Ts;             // sample period [s]
} VelocityFilter;


//Externs
extern volatile PosCtrlHandle SCurveTrajectory;
extern volatile VelocityFilter motorTracker; // Array to hold setpoints
extern volatile bool newSetpointDetected = false; // Updated in CAN processing
static unsigned int sample_count = 0; // Helps tell you how many iterations the 1kHz loop has gone through
extern float positionSetpoint; // This is updated in can_processing


//Relevent Function Prototypes
void STrajectoryInit(PosCtrlHandle *pHandle);


//Helpers
float getAMax(float motorTorque, float motorMomentIntertia);
float getJMax(float AMax, float timeToAMax) ;
float getEstimatedTrajectoryTime (float currentAngle, float targetAngle, float angularVelocity);
void computeTrajectoryParameters(PosCtrlHandle *pHandle, float startingAngle, float movementDuration, float totalAngleMovement);
float degreesToRad(float positionDegrees);
float radToDegrees(float positionRad);