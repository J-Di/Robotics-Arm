/**
 * @file    esc_sensors.h
 * @brief   Centralized sensor data cache for the B-G431B-ESC1
 *
 * All hardware reads (ADC, encoder, motor state machine) are performed
 * once per millisecond by ESC_Sensors_Update().  The resulting snapshot
 * is stored in a static struct and made available through ESC_Sensors_Get().
 *
 * Both the CAN telemetry scheduler (periodic broadcasts) and the
 * CAN request-response handler (on-demand reads) pull from this same
 * cache.  This avoids duplicate ADC conversions, keeps both paths
 * consistent, and puts all hardware-access code in one place.
 *
 * Integration:
 *   Call ESC_Sensors_Update() once per millisecond from the 1 kHz
 *   tick handler, BEFORE Telemetry_Tick_1ms() and before processing
 *   any CAN read commands for that tick.
 */

#ifndef ESC_SENSORS_H
#define ESC_SENSORS_H

#include <stdint.h>

/* -- Sensor snapshot structure */

typedef struct {
    /* Motor / motion */
    float motor_speed;         /* rad/s or RPM depending on config */
    float position;            /* rotor / joint position in rad    */

    /* Electrical */
    float bus_voltage;         /* DC bus voltage in volts          */
    float phase_current_u;     /* phase U current in amps         */
    float phase_current_v;     /* phase V current in amps         */

    /* Thermal */
    float temperature;         /* board / FET temperature in °C   */

    /* State */
    uint8_t fault_flags;       /* bitmask of active faults        */
    uint8_t motor_state;       /* idle, running, fault, etc.      */
    uint8_t control_mode;      /* FOC, 6-step, open loop, etc.    */
} ESC_SensorData;

/*  Public API */

/**
 * @brief  Read all hardware sensors and update the internal cache.
 *         Call exactly once per millisecond, before telemetry and
 *         CAN processing run for that tick.
 */
void ESC_Sensors_Update(void);

/**
 * @brief  Return a pointer to the latest sensor snapshot.
 *         The data is valid until the next call to ESC_Sensors_Update().
 * @return Pointer to the static ESC_SensorData (never NULL).
 */
const ESC_SensorData* ESC_Sensors_Get(void);

#endif /* ESC_SENSORS_H */
