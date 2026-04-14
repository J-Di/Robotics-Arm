/**
 * @file    CAN_processing_v2.c
 * @brief   CAN FD message parsing, response transmission, and data extraction
 *
 * This module handles the request-response path: the master (CANable)
 * sends a command frame, the ESC parses the 11-bit ID, dispatches to
 * the appropriate handler, and (for READ commands) sends back a
 * response containing the requested value.
 *
 * Sensor data for read responses comes from the shared ESC_Sensors
 * cache (esc_sensors.h) rather than reading hardware directly.  This
 * keeps the reads consistent with the telemetry scheduler and avoids
 * duplicate ADC conversions.
 *
 * CAN ID construction uses CAN_BuildID() from can_common.h — the
 * same builder the telemetry scheduler uses.
 *
 * Made by: James Di Sciullo
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "CAN_processing_v2.h"
#include "esc_sensors.h"
#include "main.h"
#include "mc_api.h"
#include "mc_interface.h"

#include "planner.h"
#include "SCurveTrajectory.h"
#include "velocity_ctrl.h"


#ifdef USE_UART_DEBUG
  #include "uart_debugging.h"
#else
  #define uart_debug_print(...)  ((void)0)
#endif

/* External state */
extern FDCAN_HandleTypeDef hfdcan1;
extern uint8_t ESC_ID;
extern uint8_t MOTOR_TYPE;

// Variables declared over can
volatile float positionSetpoint;    // Set by CAN_processing.c
volatile bool  newSetpointDetected; // Set by CAN_processing.c

/*  Main CAN message parser  */

void CAN_Parse_MSG(FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData)
{
    uart_debug_print("Parsing the ID...\r\n");

    ParsedCANID msg;
    uint16_t id = rxHeader->Identifier & 0x07FFU;

    /* ESCs ignore messages from other ESCs */
    msg.messageSender = (Transmitter)CAN_GetSender(id);
    if (msg.messageSender == SLAVE)
        return;

    msg.commandType = (Action)CAN_GetAction(id);
    if (msg.commandType == ACTION_RUN) {
        uart_debug_print("Run Command Detected\r\n");
        msg.runSpec = (RunSpec)CAN_GetSpec(id);
    } else {
        uart_debug_print("Read Command Detected\r\n");
        msg.readSpec = (ReadSpec)CAN_GetSpec(id);
    }

    msg.motorType   = (MotorType)CAN_GetMotorType(id);
    msg.motorConfig = (MotorConfig)CAN_GetConfig(id);

    if (msg.motorConfig == SINGLE_MOTOR) {
        msg.motorID = (MotorID)CAN_GetDeviceID(id);
        if ((int)msg.motorID != ESC_ID) {
            uart_debug_print("Not my ID\r\n");
            return;
        }
        uart_debug_print("Processing Single Command\r\n");
        Process_Single_ESC_Command(&msg, rxData);
    } else {
        uart_debug_print("Processing Multiple Command\r\n");
        Process_Multiple_ESC_Command(&msg, rxData);
    }
}

/*  Single-motor command dispatch  */

void Process_Single_ESC_Command(ParsedCANID *CANMessageID, uint8_t *rxData)
{
    float information = SingleExtractFloatFromCAN(rxData);

    if (CANMessageID->commandType == ACTION_RUN) {
        Handle_Run_Command(CANMessageID, rxData, information);
    } else {
        Handle_Read_Command(CANMessageID, rxData);
    }
}

/* Multiple-motor command dispatch  */

void Process_Multiple_ESC_Command(ParsedCANID *CANMessageID, uint8_t *rxData)
{
    uart_debug_print("Running Multiple Motors...\r\n");

    float information = extract_multiple_positions_arm(rxData);

    if (CANMessageID->commandType == ACTION_RUN) {
        Handle_Run_Command(CANMessageID, rxData, information);
    } else {
        Handle_Read_Command(CANMessageID, rxData);
    }
}

/* Run command handler*/

void Handle_Run_Command(ParsedCANID *id, uint8_t *rxData, float info)
{
    (void)rxData;
	float information = SingleExtractFloatFromCAN(rxData); // infor for now always comes as 4 bytes

    uart_debug_print("Handle_Run_Command: runSpec=%d, info=%d\r\n",(int)id->runSpec, (int)info);

    // Temporary calibration auto
    if (MC_GetSTMStateMotor1() == IDLE){
    	MC_StartMotor1();
    	return;
    }

    switch (id->runSpec) {
    case RUN_STOP:
    	MC_StopMotor1();
        uart_debug_print("  -> STOP\r\n");
        break;

    case RUN_ACKNOWLEDGE_FAULTS:
        uart_debug_print("  -> ACK FAULTS\r\n");
        break;

    case RUN_SPEED:
        if (controlMode != MODE_VELOCITY) {
            velCtrlStart(velCtrl, (VelocityFilter *)motorTracker);
            controlMode = MODE_VELOCITY;
        }
        velCtrlSetDemand(velCtrl, information);  // value in rad/s
        uart_debug_print("  -> SPEED: %d RPM\r\n", (int)info);
        break;

    case RUN_POSITION:

		/* Switch to position mode and plan a move */
		positionSetpoint = degreesToRad(information);
		newSetpointDetected = true;
		controlMode = MODE_POSITION;
        uart_debug_print("  -> POSITION: %d deg\r\n", (int)info);
        break;

    case RUN_CALIBRATION:
        uart_debug_print("  -> CALIBRATION\r\n");
        break;

    default:
        uart_debug_print("  -> UNKNOWN runSpec\r\n");
        break;
    }
}

/* Read command handler*/
/* Reads come from the shared sensor cache so values are consistent       */
/* with what the telemetry scheduler broadcasts.                          */

void Handle_Read_Command(ParsedCANID *id, uint8_t *rxData)
{
    (void)rxData;

    const ESC_SensorData *s = ESC_Sensors_Get();
    float response = 0.0f;

    uart_debug_print("Handle_Read_Command: readSpec=%d\r\n",
                     (int)id->readSpec);

    switch (id->readSpec) {
    case READ_SPEED:
        uart_debug_print("  -> READ_SPEED\r\n");
        response = s->motor_speed;
        break;
    case READ_POSITION:
        uart_debug_print("  -> READ_POSITION\r\n");
        response = radToDegrees(s->position);
        break;
    case READ_VOLTAGE:
        uart_debug_print("  -> VOLTAGE\r\n");
        response = s->bus_voltage;
        break;
    case READ_CURRENT:
        uart_debug_print("  -> CURRENT\r\n");
        response = s->phase_current_u;
        break;
    case READ_CURRENT_STATE:
        uart_debug_print("  -> CURRENT_STATE\r\n");
        response = (float)s->motor_state;
        break;
    case READ_TEMPERATURE:
        uart_debug_print("  -> TEMPERATURE\r\n");
        response = s->temperature;
        break;
    case READ_PING:
        uart_debug_print("  -> PING\r\n");
        response = 69.0f;
        break;
    case READ_CONTROL_MODE:
        uart_debug_print("  -> CONTROL_MODE\r\n");
        response = (float)s->control_mode;
        break;
    default:
        uart_debug_print("  -> UNKNOWN readSpec\r\n");
        break;
    }

    sendCANResponse(id, response);
}

/*  CAN FD response transmission */
/* Uses the shared CAN_BuildID() so the ID encoding stays in one place.   */

void sendCANResponse(ParsedCANID *CANMessageID, float information)
{
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t txData[64] = {0};

    uint8_t spec = (CANMessageID->commandType == ACTION_READ)
                   ? (uint8_t)(CANMessageID->readSpec)
                   : (uint8_t)(CANMessageID->runSpec);

    uint16_t txID = CAN_BuildID(
        SLAVE,                                      /* Sender    */
        (uint8_t)CANMessageID->commandType,         /* Action    */
        (uint8_t)CANMessageID->motorConfig,         /* Config    */
        (uint8_t)CANMessageID->motorType,           /* MotorType */
        spec,                                       /* Spec      */
        (uint8_t)CANMessageID->motorID              /* DeviceID  */
    );

    memcpy(txData, &information, sizeof(float));

    txHeader.Identifier          = txID;
    txHeader.IdType              = FDCAN_CLASSIC_CAN; // Using NOT FD here:
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;
    txHeader.DataLength          = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = FDCAN_BRS_OFF;  // Using NOT FD here:
    txHeader.FDFormat            = FDCAN_FD_CAN;
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0;

    uart_debug_print("CAN FD Response Sent!\r\n");

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK) {
        Error_Handler();
    }
}

/* - Data extraction utilities*/

float SingleExtractFloatFromCAN(uint8_t *data)
{
    float value;
    memcpy(&value, data, sizeof(float));
    return value;
}

int16_t extract_multiple_speeds(const uint8_t *rxData)
{
    uint16_t offset = (uint16_t)(ESC_ID * 2);
    int16_t value = (int16_t)((rxData[offset + 1] << 8) | rxData[offset]);
    return value;
}

float half_to_float(uint16_t half)
{
    uint32_t sign = (half >> 15) & 0x00000001U;
    uint32_t exp  = (half >> 10) & 0x0000001FU;
    uint32_t mant = half & 0x000003FFU;
    uint32_t f;

    if (exp == 0) {
        if (mant == 0) {
            f = sign << 31;
        } else {
            while ((mant & 0x00000400U) == 0) {
                mant <<= 1;
                exp  -= 1;
            }
            exp  += 1;
            mant &= ~0x00000400U;
            f = (sign << 31) | ((exp + 112) << 23) | (mant << 13);
        }
    } else if (exp == 31) {
        f = (sign << 31) | 0x7F800000U | (mant << 13);
    } else {
        f = (sign << 31) | ((exp + 112) << 23) | (mant << 13);
    }

    float result;
    memcpy(&result, &f, sizeof(result));
    return result;
}

float extract_multiple_positions_arm(const uint8_t *rxData)
{
    int index = ESC_ID - 8;
    if (index < 0 || index > 2)
        return 0.0f;

    uint16_t raw_half = (uint16_t)(rxData[index * 2 + 1] << 8)
                        | rxData[index * 2];
    return half_to_float(raw_half);
}
