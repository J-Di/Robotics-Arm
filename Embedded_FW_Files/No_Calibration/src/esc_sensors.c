/**
 * @file    esc_sensors.c
 * @brief   Centralized hardware sensor reads for the B-G431B-ESC1
 *
 * All ADC conversions, encoder reads, and motor state queries live here.
 * ESC_Sensors_Update() is called once per millisecond and populates a
 * static snapshot that both the telemetry scheduler and the
 * request-response handler consume.
 */

#include "esc_sensors.h"
#include "main.h"
#include "mc_api.h"
#include "mc_interface.h"
#include "mc_config_common.h"
#include <math.h>
#include "planner.h"
#include "SCurveTrajectory.h"
#include "velocity_ctrl.h"

/* ADC handle — configured by CubeMX with two ranks:
 *   Rank 1 = CH1  (VBUS, PA0)
 *   Rank 2 = CH5  (Temperature NTC, PB14)
 * DataAlign = LEFT, Resolution = 12-bit */
extern ADC_HandleTypeDef hadc1;

/*  Internal helpers  */

static float read_motor_state(void)
{
    /* Placeholder wire to MC_GetMecSpeedAverageMotor1() */
    return MC_GetSTMStateMotor1();
}


/*  Static sensor cache  */

static ESC_SensorData sensor_cache;

/*  Public API  */

void ESC_Sensors_Update(void)
{
    /* Electrical and thermal */
	sensor_cache.bus_voltage = VBS_GetAvBusVoltage_V(&BusVoltageSensor_M1._Super);
    sensor_cache.temperature = NTC_GetAvTemp_C(&TempSensor_M1);

    /* Motion */
    sensor_cache.motor_speed = ((VelocityFilter *)motorTracker)->omega;
    sensor_cache.position    = radToDegrees(((VelocityFilter *)motorTracker)->theta);

    /* Phase currents */
    /* TODO: read from shunt ADC / op-amp outputs */
    sensor_cache.phase_current_u = ((float)MC_GetPhaseCurrentAmplitudeMotor1())/1000;
    sensor_cache.phase_current_v = 0.0f;

    /* Motor state machine */
    sensor_cache.fault_flags  = 0;  /* TODO: GetFaultFlags()  */
    sensor_cache.motor_state  = read_motor_state();
    sensor_cache.control_mode = 0;  /* TODO: GetControlMode() */
}

const ESC_SensorData* ESC_Sensors_Get(void)
{
    return &sensor_cache;
}
