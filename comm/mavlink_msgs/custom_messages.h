/**
 * @file custom_messages.h
 * @brief Custom MAVLink message definitions
 *
 * Project-specific messages not in standard MAVLink.
 * GCS (QGC, etc.) must be built with the same XML to recognize these.
 *
 * ID range: 150000~ (development use)
 */

#ifndef CUSTOM_MESSAGES_H
#define CUSTOM_MESSAGES_H

#include <stdint.h>

/* ══════════════════════════════════════════════════════
 *  CONTROLLER_SWITCH (ID: 150000)
 *  GCS -> FC: controller switch command
 * ══════════════════════════════════════════════════════ */

/*
 * Payload (1 byte):
 *   [0] target_type — CTRL_TYPE_PID(0), CTRL_TYPE_LQR(1), CTRL_TYPE_INDI(2)
 */

typedef struct
{
    uint8_t target_type;    /* controller_type_t */
} mavlink_ctrl_switch_t;

#define MAVLINK_MSG_CTRL_SWITCH_LEN     1

/* ══════════════════════════════════════════════════════
 *  CONTROLLER_STATUS (ID: 150001)
 *  FC -> GCS: current controller status report
 * ══════════════════════════════════════════════════════ */

/*
 * Payload (13 bytes):
 *   [0..3]  roll_output  (float, -1~+1)
 *   [4..7]  pitch_output (float, -1~+1)
 *   [8..11] yaw_output   (float, -1~+1)
 *   [12]    ctrl_type    (uint8_t, CTRL_TYPE_PID/LQR/INDI)
 */

typedef struct
{
    float roll_output;
    float pitch_output;
    float yaw_output;
    uint8_t ctrl_type;      /* controller_type_t */
} mavlink_ctrl_status_t;

#define MAVLINK_MSG_CTRL_STATUS_LEN     13

#endif /* CUSTOM_MESSAGES_H */
