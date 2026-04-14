/**
 * @file    can_telemetry.h
 * @brief   Table-driven periodic CAN FD telemetry scheduler
 *
 * Each telemetry signal is a row in a configuration table.  A 1 kHz
 * tick walks the table, counts down each signal's interval, and
 * transmits when due.
 *
 * CAN IDs are built through CAN_BuildID() in can_common.h, using
 * the same 11-bit encoding as the request-response path.  The master
 * sees identical ID formats regardless of whether a frame was
 * solicited or broadcast by the scheduler.
 *
 * Sensor data comes from esc_sensors.h — the pack functions read the
 * shared cache rather than touching hardware directly.
 */

#ifndef CAN_TELEMETRY_H
#define CAN_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "can_common.h"

/*  Pack function signature  */

/** Reads from the sensor cache into tx_data[].
 *  Returns number of bytes written (informational; DLC comes from
 *  the table entry). */
typedef uint8_t (*TelemetryPackFn)(uint8_t *tx_data);

/* Telemetry table entry */

typedef struct {
    const char      *name;          /* Human-readable label */
    ReadSpec         read_spec;     /* Which ReadSpec value (bits 6:4)      */
    uint16_t         period_ms;     /* Transmission interval in ms         */
    uint16_t         countdown_ms;  /* Runtime counter (decremented/tick)   */
    uint32_t         fdcan_dlc;     /* FDCAN_DLC_BYTES_8, _12, etc.        */
    TelemetryPackFn  pack;          /* Payload builder function            */
    bool             enabled;       /* Can be toggled at runtime           */
} TelemetryEntry;

/* Public API*/

/**
 * @brief  Call exactly once per millisecond (e.g., from TIM6 ISR or
 *         from the 1 kHz flag handler in the main loop).
 *
 * Walks the telemetry table.  For each enabled entry whose countdown
 * has expired, it packs the payload and queues a CAN FD TX frame.
 *
 * IMPORTANT: Call ESC_Sensors_Update() before this function each tick
 * so the pack functions see fresh data.
 */
void Telemetry_Tick_1ms(void);

/**
 * @brief  Enable or disable a specific telemetry signal at runtime.
 * @param  spec    Identifies the signal.
 * @param  enable  true = start transmitting, false = stop.
 */
void Telemetry_SetEnabled(ReadSpec spec, bool enable);

/**
 * @brief  Change the transmission rate of a signal at runtime.
 * @param  spec       Identifies the signal.
 * @param  period_ms  New interval in milliseconds.
 */
void Telemetry_SetPeriod(ReadSpec spec, uint16_t period_ms);

#endif /* CAN_TELEMETRY_H */
