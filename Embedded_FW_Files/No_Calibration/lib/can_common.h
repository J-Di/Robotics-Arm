/**
 * @file    can_common.h
 * @brief   Shared CAN protocol definitions, enums, and ID construction
 *
 * This header is the single source of truth for the 11-bit CAN ID
 * bit-field layout and all specification enums.  Both CAN_processing_v2
 * (request-response) and can_telemetry (scheduled broadcast) include
 * this file instead of defining their own copies.
 *
 * 11-bit standard CAN ID layout:
 *
 *   Bit:  [10]    [9]     [8]       [7]        [6:4]          [3:0]
 *         Sender  Action  Config    MotorType  Specification  DeviceID
 *
 *   Sender     : 0 = Master,  1 = Slave/ESC
 *   Action     : 0 = Run,     1 = Read
 *   Config     : 0 = Multi,   1 = Single
 *   MotorType  : 0 = Drive,   1 = Steering
 *   Spec       : RunSpec (Action=0) or ReadSpec (Action=1)
 *   DeviceID   : 0-15
 */

#ifndef CAN_COMMON_H
#define CAN_COMMON_H

#include <stdint.h>

/* -- Bit-field masks and shifts ----------------------------------------- */

#define CAN_SENDER_MASK         (0x0400U)
#define CAN_SENDER_SHIFT        (10U)
#define CAN_ACTION_MASK         (0x0200U)
#define CAN_ACTION_SHIFT        (9U)
#define CAN_CONFIG_MASK         (0x0100U)
#define CAN_CONFIG_SHIFT        (8U)
#define CAN_MOTORTYPE_MASK      (0x0080U)
#define CAN_MOTORTYPE_SHIFT     (7U)
#define CAN_SPEC_MASK           (0x0070U)
#define CAN_SPEC_SHIFT          (4U)
#define CAN_DEVICEID_MASK       (0x000FU)

/* -- Protocol enums ----------------------------------------------------- */

typedef enum { MASTER = 0, SLAVE = 1 }           Transmitter;
typedef enum { ACTION_RUN = 0, ACTION_READ = 1 }  Action;
typedef enum { MULTI_MOTOR = 0, SINGLE_MOTOR = 1 } MotorConfig;
typedef enum { DRIVE = 0, STEERING = 1 }          MotorType;

/**
 * RunSpec — bits [6:4] when Action = RUN.
 * Defines which run command is being issued.
 */
typedef enum {
    RUN_STOP              = 0,
    RUN_ACKNOWLEDGE_FAULTS = 1,
    RUN_SPEED             = 2,
    RUN_POSITION          = 3,
    RUN_CALIBRATION       = 4,
    RUN_RESERVED_5        = 5,
    RUN_RESERVED_6        = 6,
    RUN_RESERVED_7        = 7,
} RunSpec;

/**
 * ReadSpec — bits [6:4] when Action = READ.
 * Used by both solicited read-responses and unsolicited telemetry.
 * The master decodes the same spec value regardless of whether
 * the frame was triggered by a request or by the telemetry scheduler.
 */
typedef enum {
    READ_SPEED            = 0,   /* Motor speed (RPM or rad/s) */
    READ_POSITION         = 1,   /* Rotor / joint position     */
    READ_VOLTAGE          = 2,   /* DC bus voltage             */
    READ_CURRENT          = 3,   /* Phase currents             */
    READ_TEMPERATURE      = 4,   /* Board / FET temperature    */
    READ_CURRENT_STATE    = 5,   /* Fault flags, motor state   */
    READ_PING             = 6,   /* Ping / heartbeat           */
    READ_CONTROL_MODE     = 7,   /* Current control mode       */
} ReadSpec;

/* Keep the old name alive for CAN_processing_v2 compatibility */
#define READ_CALIBRATION  READ_SPEED  /* legacy alias — reconsider later */

typedef enum {
    MOTOR_ID_0 = 0, MOTOR_ID_1 = 1, MOTOR_ID_2  = 2,  MOTOR_ID_3  = 3,
    MOTOR_ID_4 = 4, MOTOR_ID_5 = 5, MOTOR_ID_6  = 6,  MOTOR_ID_7  = 7,
    MOTOR_ID_8 = 8, MOTOR_ID_9 = 9, MOTOR_ID_10 = 10, MOTOR_ID_11 = 11,
} MotorID;

/* Parsed message structure */

typedef struct {
    Transmitter  messageSender;
    Action       commandType;
    MotorConfig  motorConfig;
    MotorType    motorType;
    MotorID      motorID;
    union {
        RunSpec  runSpec;
        ReadSpec readSpec;
    };
} ParsedCANID;

/* CAN ID construction  */

/**
 * @brief  Build a complete 11-bit CAN ID from individual fields.
 *
 * This is the single canonical builder used by both sendCANResponse()
 * and the telemetry scheduler.
 */
static inline uint16_t CAN_BuildID(uint8_t sender,
                                   uint8_t action,
                                   uint8_t config,
                                   uint8_t motor_type,
                                   uint8_t spec,
                                   uint8_t device_id)
{
    uint16_t id = 0;
    id |= (uint16_t)(sender     & 0x01U) << CAN_SENDER_SHIFT;
    id |= (uint16_t)(action     & 0x01U) << CAN_ACTION_SHIFT;
    id |= (uint16_t)(config     & 0x01U) << CAN_CONFIG_SHIFT;
    id |= (uint16_t)(motor_type & 0x01U) << CAN_MOTORTYPE_SHIFT;
    id |= (uint16_t)(spec       & 0x07U) << CAN_SPEC_SHIFT;
    id |= (uint16_t)(device_id  & 0x0FU);
    return id;
}

/*  CAN ID field extractors */

static inline uint8_t CAN_GetSender(uint16_t id)    { return (id & CAN_SENDER_MASK)    >> CAN_SENDER_SHIFT;    }
static inline uint8_t CAN_GetAction(uint16_t id)    { return (id & CAN_ACTION_MASK)    >> CAN_ACTION_SHIFT;    }
static inline uint8_t CAN_GetConfig(uint16_t id)    { return (id & CAN_CONFIG_MASK)    >> CAN_CONFIG_SHIFT;    }
static inline uint8_t CAN_GetMotorType(uint16_t id) { return (id & CAN_MOTORTYPE_MASK) >> CAN_MOTORTYPE_SHIFT; }
static inline uint8_t CAN_GetSpec(uint16_t id)      { return (id & CAN_SPEC_MASK)      >> CAN_SPEC_SHIFT;      }
static inline uint8_t CAN_GetDeviceID(uint16_t id)  { return (id & CAN_DEVICEID_MASK);                         }

#endif /* CAN_COMMON_H */
