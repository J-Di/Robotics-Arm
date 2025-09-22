#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//These are for the elbow motor, and will need to make a helper function to extract for rover depending on esc
#define MOTOR_TORQUE  405e-3  // Nm
#define MOTOR_MOMENT 13.31e-6 // kg·m²
//

typedef struct
{
  float MovementDuration;              /**< @brief Total duration of the programmed movement */
  float StartingAngle;                 /**< @brief Current mechanical position */
  float FinalAngle;                    /**< @brief Target mechanical position including start position */
  float AngleStep;                     /**< @brief Target mechanical position */
  float SubStep[6];                    /**< @brief Sub step interval time of acceleration and deceleration phases */
  float SubStepDuration;               /**< @brief Sub step time duration of sequence : acceleration / cruise /
                                                   deceleration */
  float ElapseTime;                    /**< @brief Elapse time during trajectory movement execution */
  float SamplingTime;                  /**< @brief Sampling time at which the movement regulation is called
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

} PosCtrlHandle;



//Relevent Function Prototypes


//Helpers
float getAMax(float motorTorque, float motorMomentIntertia);
float getJMax(float AMax, float timeToAMax) ;
