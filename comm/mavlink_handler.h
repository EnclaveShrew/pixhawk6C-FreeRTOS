/**
 * @file mavlink_handler.h
 * @brief MAVLink v2 protocol handler
 *
 * MAVLink v2 패킷 구조:
 *   [STX 0xFD] [LEN] [INC_FLAGS] [CMP_FLAGS] [SEQ] [SYS_ID] [COMP_ID]
 *   [MSG_ID_L] [MSG_ID_M] [MSG_ID_H] [PAYLOAD...] [CRC_L] [CRC_H]
 *
 * 지원 메시지:
 *   송신: HEARTBEAT(0), ATTITUDE(30), GPS_RAW_INT(24), SYS_STATUS(1)
 *   수신: COMMAND_LONG(76), PARAM_SET(23)
 *   커스텀: CONTROLLER_SWITCH(150000), CONTROLLER_STATUS(150001)
 */

#ifndef MAVLINK_HANDLER_H
#define MAVLINK_HANDLER_H

#include "uart_wrapper.h"
#include "sensor_types.h"
#include "controller_interface.h"
#include <stdint.h>

/* ══════════════════════════════════════════════════════
 *  MAVLink v2 constants
 * ══════════════════════════════════════════════════════ */

#define MAVLINK_STX_V2          0xFD
#define MAVLINK_HEADER_LEN      10
#define MAVLINK_CRC_LEN         2
#define MAVLINK_MAX_PAYLOAD     128

/* System / Component ID */
#define MAVLINK_SYS_ID          1
#define MAVLINK_COMP_ID         1

/* Standard message IDs */
#define MAVLINK_MSG_HEARTBEAT       0
#define MAVLINK_MSG_SYS_STATUS      1
#define MAVLINK_MSG_PARAM_SET       23
#define MAVLINK_MSG_GPS_RAW_INT     24
#define MAVLINK_MSG_ATTITUDE        30
#define MAVLINK_MSG_COMMAND_LONG    76
#define MAVLINK_MSG_COMMAND_ACK     77

/* Custom message IDs (150000~) */
#define MAVLINK_MSG_CTRL_SWITCH     150000
#define MAVLINK_MSG_CTRL_STATUS     150001

/* MAV_TYPE */
#define MAV_TYPE_QUADROTOR      2

/* MAV_AUTOPILOT */
#define MAV_AUTOPILOT_GENERIC   0

/* MAV_STATE */
#define MAV_STATE_STANDBY       3
#define MAV_STATE_ACTIVE        4

/* MAV_MODE_FLAG */
#define MAV_MODE_FLAG_CUSTOM    1
#define MAV_MODE_FLAG_STABILIZE 16

/* ══════════════════════════════════════════════════════
 *  MAVLink parser state machine
 * ══════════════════════════════════════════════════════ */

typedef enum
{
    MAV_STATE_WAIT_STX,
    MAV_STATE_LEN,
    MAV_STATE_INC_FLAGS,
    MAV_STATE_CMP_FLAGS,
    MAV_STATE_SEQ,
    MAV_STATE_SYS_ID,
    MAV_STATE_COMP_ID,
    MAV_STATE_MSG_ID_L,
    MAV_STATE_MSG_ID_M,
    MAV_STATE_MSG_ID_H,
    MAV_STATE_PAYLOAD,
    MAV_STATE_CRC_L,
    MAV_STATE_CRC_H,
} mavlink_parse_state_t;

/* ══════════════════════════════════════════════════════
 *  MAVLink parsed message
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    uint32_t msg_id;
    uint8_t sys_id;
    uint8_t comp_id;
    uint8_t seq;
    uint8_t payload_len;
    uint8_t payload[MAVLINK_MAX_PAYLOAD];
} mavlink_message_t;

/* ══════════════════════════════════════════════════════
 *  MAVLink handler state
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    uart_dev_t *uart_dev;

    /* Parser */
    mavlink_parse_state_t parse_state;
    mavlink_message_t rx_msg;
    uint8_t rx_payload_idx;
    uint16_t rx_crc;

    /* TX sequence counter */
    uint8_t tx_seq;

    /* Callback: 수신된 메시지 처리 */
    void (*on_message)(const mavlink_message_t *msg);
} mavlink_handler_t;

/* ══════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════ */

/**
 * @brief Initialize MAVLink handler
 * @param mav         Handler state
 * @param uart_dev    UART device for MAVLink communication
 * @param on_message  Callback for received messages (nullable)
 * @return 0 on success
 */
int mavlink_init(mavlink_handler_t *mav, uart_dev_t *uart_dev,
                 void (*on_message)(const mavlink_message_t *msg));

/**
 * @brief Poll and parse incoming MAVLink data
 * @param mav  Handler state
 * @return Number of complete messages parsed, 0 if none
 */
int mavlink_poll(mavlink_handler_t *mav);

/**
 * @brief Send HEARTBEAT message (1 Hz)
 * @param mav        Handler state
 * @param base_mode  MAV_MODE_FLAG bitmask
 * @param state      MAV_STATE enum
 */
int mavlink_send_heartbeat(mavlink_handler_t *mav, uint8_t base_mode, uint8_t state);

/**
 * @brief Send ATTITUDE message (10~50 Hz)
 * @param mav    Handler state
 * @param roll   Roll angle (rad)
 * @param pitch  Pitch angle (rad)
 * @param yaw    Yaw angle (rad)
 * @param p      Roll rate (rad/s)
 * @param q      Pitch rate (rad/s)
 * @param r      Yaw rate (rad/s)
 */
int mavlink_send_attitude(mavlink_handler_t *mav,
                          float roll, float pitch, float yaw,
                          float p, float q, float r);

/**
 * @brief Send GPS_RAW_INT message (1~5 Hz)
 * @param mav       Handler state
 * @param fix_type  GPS fix type (0~5)
 * @param lat       Latitude (1e-7 deg)
 * @param lon       Longitude (1e-7 deg)
 * @param alt       Altitude MSL (mm)
 * @param num_sv    Number of satellites
 * @param h_acc     Horizontal accuracy (mm)
 * @param v_acc     Vertical accuracy (mm)
 */
int mavlink_send_gps_raw(mavlink_handler_t *mav,
                         uint8_t fix_type, int32_t lat, int32_t lon,
                         int32_t alt, uint8_t num_sv,
                         uint32_t h_acc, uint32_t v_acc);

/**
 * @brief Send SYS_STATUS message (1 Hz)
 * @param mav           Handler state
 * @param battery_mv    Battery voltage (mV)
 * @param battery_pct   Battery remaining (0~100, -1 if unknown)
 * @param cpu_load      CPU load (0~1000, permille)
 */
int mavlink_send_sys_status(mavlink_handler_t *mav,
                            uint16_t battery_mv, int8_t battery_pct,
                            uint16_t cpu_load);

/**
 * @brief Send COMMAND_ACK message
 * @param mav      Handler state
 * @param command  Command being acknowledged
 * @param result   0=accepted, 1~=error
 */
int mavlink_send_command_ack(mavlink_handler_t *mav,
                             uint16_t command, uint8_t result);

/**
 * @brief Send custom CONTROLLER_STATUS message
 * @param mav             Handler state
 * @param ctrl_type       Active controller type (CTRL_TYPE_PID/LQR/INDI)
 * @param roll_output     Current roll control output
 * @param pitch_output    Current pitch control output
 * @param yaw_output      Current yaw control output
 */
int mavlink_send_ctrl_status(mavlink_handler_t *mav,
                             uint8_t ctrl_type,
                             float roll_output, float pitch_output,
                             float yaw_output);

#endif /* MAVLINK_HANDLER_H */
