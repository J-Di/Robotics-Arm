/**
 * @file    CAN_processing_v2.h
 * @brief   CAN FD message parsing, command dispatch, and response TX
 *
 * This module handles the request-response side of the CAN protocol:
 * the master sends a command, the ESC parses it, executes (or stubs)
 * the action, and sends back a response.
 *
 * All protocol enums, bit-field definitions, and the CAN ID builder
 * live in can_common.h.  This header only declares the functions
 * specific to RX parsing and TX response.
 */

#ifndef CAN_PROCESSING_V2_H
#define CAN_PROCESSING_V2_H

#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "can_common.h"
#include "main.h"
#include "velocity_ctrl.h"

// Externs
extern volatile ControlMode_t controlMode;
extern VelCtrlHandle *velCtrl;
extern volatile bool  newSetpointDetected; // Set by CAN_processing.c
extern volatile float positionSetpoint;    // Set by CAN_processing.c

/*  Public API  */

/** Parse a received CAN FD frame and dispatch to the appropriate handler. */
void CAN_Parse_MSG(FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);

/** Process a command addressed to this specific ESC (single-motor). */
void Process_Single_ESC_Command(ParsedCANID *CANMessageID, uint8_t *rxData);

/** Process a broadcast command addressed to multiple ESCs. */
void Process_Multiple_ESC_Command(ParsedCANID *CANMessageID, uint8_t *rxData);

/** Handle an ACTION_RUN command (stop, speed, position, etc.). */
void Handle_Run_Command(ParsedCANID *id, uint8_t *rxData, float info);

/** Handle an ACTION_READ command and send back the requested value. */
void Handle_Read_Command(ParsedCANID *id, uint8_t *rxData);

/** Build and transmit a CAN FD response frame to the master. */
void sendCANResponse(ParsedCANID *CANMessageID, float information);

/*  Data extraction utilities  */

float   SingleExtractFloatFromCAN(uint8_t *data);
int16_t extract_multiple_speeds(const uint8_t *rxData);
float   extract_multiple_positions_arm(const uint8_t *rxData);
float   half_to_float(uint16_t half);


#endif /* CAN_PROCESSING_V2_H */
