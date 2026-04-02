/**
 * @file mavlink_handler.c
 * @brief MAVLink v2 protocol handler implementation
 *
 * MAVLink v2 CRC: CRC-16/MCRF4XX + CRC_EXTRA (per-message seed)
 * Implemented directly without external MAVLink library dependency.
 */

#include "mavlink_handler.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stddef.h>

/* ── CRC-16/MCRF4XX ────────────────────────────────── */

static void crc_init(uint16_t *crc)
{
    *crc = 0xFFFF;
}

static void crc_accumulate(uint16_t *crc, uint8_t byte)
{
    uint8_t tmp;
    tmp = byte ^ (uint8_t)(*crc & 0xFF);
    tmp ^= (tmp << 4);
    *crc = (*crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
}

static void crc_accumulate_buf(uint16_t *crc, const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        crc_accumulate(crc, buf[i]);
    }
}

/*
 * CRC_EXTRA: hash of each message's field definitions.
 * Verifies both sides use the same message structure.
 * Only messages used in this project are defined here.
 */
/*
 * CRC_EXTRA for custom messages:
 *   Computed from field type+name per MAVLink spec.
 *   CTRL_SWITCH (150000): uint8_t target_type -> 0xBE (190)
 *   CTRL_STATUS (150001): float roll_output, float pitch_output,
 *                          float yaw_output, uint8_t ctrl_type -> 0x4D (77)
 */
#define CRC_EXTRA_CTRL_SWITCH   190
#define CRC_EXTRA_CTRL_STATUS   77

static uint8_t mavlink_get_crc_extra(uint32_t msg_id)
{
    switch (msg_id)
    {
    case MAVLINK_MSG_HEARTBEAT:     return 50;
    case MAVLINK_MSG_SYS_STATUS:    return 124;
    case MAVLINK_MSG_GPS_RAW_INT:   return 24;
    case MAVLINK_MSG_ATTITUDE:      return 39;
    case MAVLINK_MSG_COMMAND_LONG:   return 152;
    case MAVLINK_MSG_COMMAND_ACK:    return 143;
    case MAVLINK_MSG_PARAM_SET:      return 168;
    case MAVLINK_MSG_CTRL_SWITCH:    return CRC_EXTRA_CTRL_SWITCH;
    case MAVLINK_MSG_CTRL_STATUS:    return CRC_EXTRA_CTRL_STATUS;
    default:                         return 0;
    }
}

/* ── Internal: send packet ────────────────────────── */

static int mavlink_send_packet(mavlink_handler_t *mav, uint32_t msg_id,
                               const uint8_t *payload, uint8_t payload_len)
{
    /* Header (10 bytes) */
    uint8_t header[MAVLINK_HEADER_LEN];
    header[0] = MAVLINK_STX_V2;
    header[1] = payload_len;
    header[2] = 0;  /* incompat flags */
    header[3] = 0;  /* compat flags */
    header[4] = mav->tx_seq++;
    header[5] = MAVLINK_SYS_ID;
    header[6] = MAVLINK_COMP_ID;
    header[7] = (uint8_t)(msg_id & 0xFF);
    header[8] = (uint8_t)((msg_id >> 8) & 0xFF);
    header[9] = (uint8_t)((msg_id >> 16) & 0xFF);

    /* CRC: header[1..9] + payload + crc_extra */
    uint16_t crc;
    crc_init(&crc);
    crc_accumulate_buf(&crc, &header[1], 9);
    crc_accumulate_buf(&crc, payload, payload_len);
    crc_accumulate(&crc, mavlink_get_crc_extra(msg_id));

    uint8_t crc_buf[2] = { (uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8) };

    /* Send: header + payload + crc */
    if (uart_send(mav->uart_dev, header, MAVLINK_HEADER_LEN) < 0)
    {
        return -1;
    }
    if (payload_len > 0 && uart_send(mav->uart_dev, payload, payload_len) < 0)
    {
        return -1;
    }
    if (uart_send(mav->uart_dev, crc_buf, 2) < 0)
    {
        return -1;
    }

    return 0;
}

/* ── Internal: little-endian helpers ───────────────── */

static void put_float(uint8_t *buf, float val)
{
    memcpy(buf, &val, 4);
}

static void put_u32(uint8_t *buf, uint32_t val)
{
    buf[0] = (uint8_t)(val);
    buf[1] = (uint8_t)(val >> 8);
    buf[2] = (uint8_t)(val >> 16);
    buf[3] = (uint8_t)(val >> 24);
}

static void put_i32(uint8_t *buf, int32_t val)
{
    put_u32(buf, (uint32_t)val);
}

static void put_u16(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)(val);
    buf[1] = (uint8_t)(val >> 8);
}

/* ── Initialization ─────────────────────────────────── */

int mavlink_init(mavlink_handler_t *mav, uart_dev_t *uart_dev,
                 void (*on_message)(const mavlink_message_t *msg))
{
    memset(mav, 0, sizeof(mavlink_handler_t));
    mav->uart_dev = uart_dev;
    mav->parse_state = MAV_STATE_WAIT_STX;
    mav->on_message = on_message;
    return 0;
}

/* ── Byte-level parser ─────────────────────────────── */

#define MAVLINK_PARSE_TIMEOUT_MS  200

static int mavlink_parse_byte(mavlink_handler_t *mav, uint8_t byte)
{
    /* Timeout: if mid-packet and no byte for too long, reset parser */
    if (mav->parse_state != MAV_STATE_WAIT_STX)
    {
        uint32_t now = HAL_GetTick();
        if ((now - mav->rx_start_tick) > MAVLINK_PARSE_TIMEOUT_MS)
        {
            mav->parse_state = MAV_STATE_WAIT_STX;
        }
    }

    switch (mav->parse_state)
    {
    case MAV_STATE_WAIT_STX:
        if (byte == MAVLINK_STX_V2)
        {
            mav->parse_state = MAV_STATE_LEN;
            crc_init(&mav->rx_crc);
            mav->rx_start_tick = HAL_GetTick();
        }
        break;

    case MAV_STATE_LEN:
        mav->rx_msg.payload_len = byte;
        crc_accumulate(&mav->rx_crc, byte);
        mav->parse_state = MAV_STATE_INC_FLAGS;
        break;

    case MAV_STATE_INC_FLAGS:
        crc_accumulate(&mav->rx_crc, byte);
        mav->parse_state = MAV_STATE_CMP_FLAGS;
        break;

    case MAV_STATE_CMP_FLAGS:
        crc_accumulate(&mav->rx_crc, byte);
        mav->parse_state = MAV_STATE_SEQ;
        break;

    case MAV_STATE_SEQ:
        mav->rx_msg.seq = byte;
        crc_accumulate(&mav->rx_crc, byte);
        mav->parse_state = MAV_STATE_SYS_ID;
        break;

    case MAV_STATE_SYS_ID:
        mav->rx_msg.sys_id = byte;
        crc_accumulate(&mav->rx_crc, byte);
        mav->parse_state = MAV_STATE_COMP_ID;
        break;

    case MAV_STATE_COMP_ID:
        mav->rx_msg.comp_id = byte;
        crc_accumulate(&mav->rx_crc, byte);
        mav->parse_state = MAV_STATE_MSG_ID_L;
        break;

    case MAV_STATE_MSG_ID_L:
        mav->rx_msg.msg_id = byte;
        crc_accumulate(&mav->rx_crc, byte);
        mav->parse_state = MAV_STATE_MSG_ID_M;
        break;

    case MAV_STATE_MSG_ID_M:
        mav->rx_msg.msg_id |= ((uint32_t)byte << 8);
        crc_accumulate(&mav->rx_crc, byte);
        mav->parse_state = MAV_STATE_MSG_ID_H;
        break;

    case MAV_STATE_MSG_ID_H:
        mav->rx_msg.msg_id |= ((uint32_t)byte << 16);
        crc_accumulate(&mav->rx_crc, byte);
        mav->rx_payload_idx = 0;

        if (mav->rx_msg.payload_len == 0)
        {
            /* Accumulate CRC_EXTRA then proceed to CRC verification */
            crc_accumulate(&mav->rx_crc, mavlink_get_crc_extra(mav->rx_msg.msg_id));
            mav->parse_state = MAV_STATE_CRC_L;
        }
        else if (mav->rx_msg.payload_len > MAVLINK_MAX_PAYLOAD)
        {
            mav->parse_state = MAV_STATE_WAIT_STX;
        }
        else
        {
            mav->parse_state = MAV_STATE_PAYLOAD;
        }
        break;

    case MAV_STATE_PAYLOAD:
        mav->rx_msg.payload[mav->rx_payload_idx++] = byte;
        crc_accumulate(&mav->rx_crc, byte);

        if (mav->rx_payload_idx >= mav->rx_msg.payload_len)
        {
            crc_accumulate(&mav->rx_crc, mavlink_get_crc_extra(mav->rx_msg.msg_id));
            mav->parse_state = MAV_STATE_CRC_L;
        }
        break;

    case MAV_STATE_CRC_L:
        if (byte == (uint8_t)(mav->rx_crc & 0xFF))
        {
            mav->parse_state = MAV_STATE_CRC_H;
        }
        else
        {
            mav->parse_state = MAV_STATE_WAIT_STX;
        }
        break;

    case MAV_STATE_CRC_H:
        mav->parse_state = MAV_STATE_WAIT_STX;
        if (byte == (uint8_t)(mav->rx_crc >> 8))
        {
            /* CRC passed — process message */
            if (mav->on_message != NULL)
            {
                mav->on_message(&mav->rx_msg);
            }
            return 1;
        }
        break;
    }

    return 0;
}

/* ── Polling receive ───────────────────────────────── */

int mavlink_poll(mavlink_handler_t *mav)
{
    uint8_t byte;
    int count = 0;

    while (uart_receive_byte(mav->uart_dev, &byte) == 0)
    {
        if (mavlink_parse_byte(mav, byte) == 1)
        {
            count++;
        }
    }

    return count;
}

/* ── HEARTBEAT (msg_id=0, 9 bytes) ─────────────────── */

int mavlink_send_heartbeat(mavlink_handler_t *mav, uint8_t base_mode, uint8_t state)
{
    uint8_t payload[9];
    memset(payload, 0, sizeof(payload));

    /* custom_mode (4 bytes) */
    put_u32(&payload[0], 0);
    /* type */
    payload[4] = MAV_TYPE_QUADROTOR;
    /* autopilot */
    payload[5] = MAV_AUTOPILOT_GENERIC;
    /* base_mode */
    payload[6] = base_mode;
    /* system_status */
    payload[7] = state;
    /* mavlink_version */
    payload[8] = 2;

    return mavlink_send_packet(mav, MAVLINK_MSG_HEARTBEAT, payload, 9);
}

/* ── ATTITUDE (msg_id=30, 28 bytes) ────────────────── */

int mavlink_send_attitude(mavlink_handler_t *mav,
                          float roll, float pitch, float yaw,
                          float p, float q, float r)
{
    uint8_t payload[28];
    memset(payload, 0, sizeof(payload));

    /* time_boot_ms (4 bytes) — QGC uses this for graph time axis */
    put_u32(&payload[0], HAL_GetTick());
    /* roll, pitch, yaw (rad) */
    put_float(&payload[4], roll);
    put_float(&payload[8], pitch);
    put_float(&payload[12], yaw);
    /* rollspeed, pitchspeed, yawspeed (rad/s) */
    put_float(&payload[16], p);
    put_float(&payload[20], q);
    put_float(&payload[24], r);

    return mavlink_send_packet(mav, MAVLINK_MSG_ATTITUDE, payload, 28);
}

/* ── GPS_RAW_INT (msg_id=24, 52 bytes) ─────────────── */

int mavlink_send_gps_raw(mavlink_handler_t *mav,
                         uint8_t fix_type, int32_t lat, int32_t lon,
                         int32_t alt, uint8_t num_sv,
                         uint32_t h_acc, uint32_t v_acc)
{
    uint8_t payload[52];
    memset(payload, 0, sizeof(payload));

    /* time_usec (8 bytes) */
    /* lat, lon (1e-7 deg) */
    put_i32(&payload[8], lat);
    put_i32(&payload[12], lon);
    /* alt (mm, AMSL) */
    put_i32(&payload[16], alt);
    /* eph (cm) — convert h_acc from mm to cm */
    put_u16(&payload[20], (uint16_t)(h_acc / 10));
    /* epv (cm) */
    put_u16(&payload[22], (uint16_t)(v_acc / 10));
    /* vel (cm/s) */
    put_u16(&payload[24], 0);
    /* cog (cdeg) */
    put_u16(&payload[26], 0);
    /* fix_type */
    payload[28] = fix_type;
    /* satellites_visible */
    payload[29] = num_sv;

    return mavlink_send_packet(mav, MAVLINK_MSG_GPS_RAW_INT, payload, 52);
}

/* ── SYS_STATUS (msg_id=1, 31 bytes) ──────────────── */

int mavlink_send_sys_status(mavlink_handler_t *mav,
                            uint16_t battery_mv, int8_t battery_pct,
                            uint16_t cpu_load)
{
    uint8_t payload[31];
    memset(payload, 0, sizeof(payload));

    /* sensors present/enabled/health (12 bytes) — all zeros */
    /* load (permille) */
    put_u16(&payload[12], cpu_load);
    /* voltage_battery (mV) */
    put_u16(&payload[14], battery_mv);
    /* current_battery (cA) — not implemented */
    put_u16(&payload[16], 0);
    /* battery_remaining (%) */
    payload[30] = (uint8_t)battery_pct;

    return mavlink_send_packet(mav, MAVLINK_MSG_SYS_STATUS, payload, 31);
}

/* ── COMMAND_ACK (msg_id=77, 3 bytes) ──────────────── */

int mavlink_send_command_ack(mavlink_handler_t *mav,
                             uint16_t command, uint8_t result)
{
    uint8_t payload[3];
    put_u16(&payload[0], command);
    payload[2] = result;

    return mavlink_send_packet(mav, MAVLINK_MSG_COMMAND_ACK, payload, 3);
}

/* ── Custom: CONTROLLER_STATUS (msg_id=150001, 13 bytes) ── */

int mavlink_send_ctrl_status(mavlink_handler_t *mav,
                             uint8_t ctrl_type,
                             float roll_output, float pitch_output,
                             float yaw_output)
{
    uint8_t payload[13];
    put_float(&payload[0], roll_output);
    put_float(&payload[4], pitch_output);
    put_float(&payload[8], yaw_output);
    payload[12] = ctrl_type;

    return mavlink_send_packet(mav, MAVLINK_MSG_CTRL_STATUS, payload, 13);
}
