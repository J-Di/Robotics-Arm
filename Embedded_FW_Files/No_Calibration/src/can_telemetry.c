/**
 * can_telemetry.c
 *
 *  Created on: Mar 30, 2026
 *      Author: james
 * @file    can_telemetry.c
 * @brief   Periodic CAN FD telemetry signal table and scheduler
 *
 * To add a new telemetry signal:
 *   1. Add a value to ReadSpec in can_common.h
 *   2. Write a pack_xxx() function below
 *   3. Add a row to telemetry_table[]
 *
 * Pack functions read from the ESC_Sensors cache (esc_sensors.h)
 * rather than touching hardware directly.  ESC_Sensors_Update()
 * must be called before Telemetry_Tick_1ms() each tick.
 *
 * Countdown staggering:
 *   Signals that share the same period are given different initial
 *   countdown values to spread TX load across ticks and avoid
 *   FIFO bursts.  See the countdown_ms column in the table.
 */

#include "can_telemetry.h"
#include "esc_sensors.h"
#include <string.h>

/* External dependencies */

extern FDCAN_HandleTypeDef hfdcan1;    /* defined in main.c by CubeMX */
extern uint8_t ESC_ID;                 /* 0–15, this device's CAN address */
extern uint8_t MOTOR_TYPE;             /* 0 = Drive, 1 = Steering */

/* -- Pack functions - */
/* Each reads from the sensor cache and fills tx_data[].                   */
/* Returns number of bytes written.                                        */

static uint8_t pack_motor_speed(uint8_t *tx_data)
{
    float speed = ESC_Sensors_Get()->motor_speed;
    memcpy(tx_data, &speed, sizeof(float));
    return 4;
}

static uint8_t pack_position(uint8_t *tx_data)
{
    float position = ESC_Sensors_Get()->position;
    memcpy(tx_data, &position, sizeof(float));
    return 4;
}

static uint8_t pack_bus_voltage(uint8_t *tx_data)
{
    float vbus = ESC_Sensors_Get()->bus_voltage;
    memcpy(tx_data, &vbus, sizeof(float));
    return 4;
}

static uint8_t pack_phase_currents(uint8_t *tx_data)
{
    const ESC_SensorData *s = ESC_Sensors_Get();
    memcpy(tx_data,     &s->phase_current_u, sizeof(float));
//    memcpy(tx_data + 4, &s->phase_current_v, sizeof(float));
    return 8;
}

static uint8_t pack_temperature(uint8_t *tx_data)
{
    float temp = ESC_Sensors_Get()->temperature;
    memcpy(tx_data, &temp, sizeof(float));
    return 4;
}

static uint8_t pack_status(uint8_t *tx_data)
{
    const ESC_SensorData *s = ESC_Sensors_Get();
    tx_data[0] = s->fault_flags;
    tx_data[1] = s->motor_state;
    tx_data[2] = s->control_mode;
    tx_data[3] = 0; /* reserved */
    return 4;
}

static uint8_t pack_ping(uint8_t *tx_data)
{
	float val = 69.0f;
	memcpy(tx_data, &val, sizeof(float));
	return 4;
}



/* -- Telemetry table  */
/*                                                                         */
/* Single source of truth for all periodic broadcasts.                     */
/*                                                                         */
/* countdown_ms values are staggered so signals with the same period       */
/* don't all fire on the same tick.  Rule of thumb: offset each by         */
/* period / (number of signals at that rate).                              */

TelemetryEntry telemetry_table[] = {
/*  name               read_spec         period  countdown  DLC                  pack                enabled */
    { "MotorSpeed",    READ_SPEED,         100,      0,     FDCAN_DLC_BYTES_8,   pack_motor_speed,   true  },
    { "Position",      READ_POSITION,      100,     33,     FDCAN_DLC_BYTES_8,   pack_position,      true  },
    { "BusVoltage",    READ_VOLTAGE,       1000,     10,     FDCAN_DLC_BYTES_8,   pack_bus_voltage,   true  },
    { "PhaseCurrents", READ_CURRENT,        100,     66,     FDCAN_DLC_BYTES_8,   pack_phase_currents,true  },
    { "Temperature",   READ_TEMPERATURE,   1000,    500,     FDCAN_DLC_BYTES_8,   pack_temperature,   true  },
    { "Status",        READ_CURRENT_STATE,  500,    250,     FDCAN_DLC_BYTES_8,   pack_status,        true  },
    { "Ping",          READ_PING,  		   1000,    750,     FDCAN_DLC_BYTES_8,   pack_ping,        	true  },
};

static const uint16_t TELEMETRY_COUNT = sizeof(telemetry_table) / sizeof(telemetry_table[0]);

/* -- Scheduler tick (1 ms)  */

void Telemetry_Tick_1ms(void)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[64];

    for (uint16_t i = 0; i < TELEMETRY_COUNT; i++)
    {
        TelemetryEntry *e = &telemetry_table[i];

        if (!e->enabled)
            continue;

        if (e->countdown_ms > 0)
        {
            e->countdown_ms--;
            continue;
        }

        /* Reload for next cycle.  -1 because this tick is tick zero
         * of the new cycle; without it the period would be off by one. */
        e->countdown_ms = e->period_ms - 1;

        /* Pack the payload */
        memset(tx_data, 0, sizeof(tx_data));
        e->pack(tx_data);

        /* Build TX header using the shared CAN ID builder */
        tx_header.Identifier = CAN_BuildID(
            SLAVE,              /* Sender   = ESC        */
            ACTION_READ,        /* Action   = READ       */
            SINGLE_MOTOR,       /* Config   = Single     */
            MOTOR_TYPE & 0x01,  /* MotorType             */
            e->read_spec,       /* ReadSpec (3 bits)     */
            ESC_ID & 0x0F       /* DeviceID              */
        );


        tx_header.IdType              = FDCAN_CLASSIC_CAN; // Using NOT FD here:
        tx_header.TxFrameType         = FDCAN_DATA_FRAME;
        tx_header.DataLength          = e->fdcan_dlc;
        tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_header.BitRateSwitch       = FDCAN_BRS_OFF;  // Using NOT FD here:
        tx_header.FDFormat            = FDCAN_FD_CAN;
        tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        tx_header.MessageMarker       = 0;

        /* Queue the frame.  If the TX FIFO is full, silently skip.
         * Telemetry is periodic so the next cycle carries fresh data. */
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
    }
}

/* -- Runtime control  */

void Telemetry_SetEnabled(ReadSpec spec, bool enable)
{
    for (uint16_t i = 0; i < TELEMETRY_COUNT; i++)
    {
        if (telemetry_table[i].read_spec == spec)
        {
            telemetry_table[i].enabled = enable;
            if (enable)
            {
                telemetry_table[i].countdown_ms =
                    telemetry_table[i].period_ms - 1;
            }
            return;
        }
    }
}

void Telemetry_SetPeriod(ReadSpec spec, uint16_t period_ms)
{
    for (uint16_t i = 0; i < TELEMETRY_COUNT; i++)
    {
        if (telemetry_table[i].read_spec == spec)
        {
            telemetry_table[i].period_ms    = period_ms;
            telemetry_table[i].countdown_ms = period_ms - 1;
            return;
        }
    }
}
