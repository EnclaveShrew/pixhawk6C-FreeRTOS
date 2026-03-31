/**
 * @file ubx_m10.h
 * @brief u-blox M10 GPS driver — UBX protocol parser
 *
 * Interface: UART (default 38400 baud)
 * Protocol: UBX binary
 * Primary message: NAV-PVT (Position Velocity Time)
 */

#ifndef UBX_M10_H
#define UBX_M10_H

#include "sensor_types.h"
#include "uart_wrapper.h"

/* ══════════════════════════════════════════════════════
 *  UBX Protocol constants
 * ══════════════════════════════════════════════════════ */

#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

/* Message classes */
#define UBX_CLASS_NAV 0x01
#define UBX_CLASS_CFG 0x06
#define UBX_CLASS_ACK 0x05

/* Message IDs */
#define UBX_NAV_PVT 0x07  /* Navigation Position Velocity Time */
#define UBX_CFG_PRT 0x00  /* Port configuration */
#define UBX_CFG_MSG 0x01  /* Message rate configuration */
#define UBX_CFG_RATE 0x08 /* Navigation rate */
#define UBX_ACK_ACK 0x01
#define UBX_ACK_NAK 0x00

/* NAV-PVT payload size */
#define UBX_NAV_PVT_LEN 92

/* Max payload size (NAV-PVT가 가장 큼) */
#define UBX_MAX_PAYLOAD 96

/* ══════════════════════════════════════════════════════
 *  GPS fix type
 * ══════════════════════════════════════════════════════ */

typedef enum
{
    GPS_FIX_NONE = 0,
    GPS_FIX_DEAD = 1, /* Dead reckoning */
    GPS_FIX_2D = 2,
    GPS_FIX_3D = 3,
    GPS_FIX_COMBINED = 4, /* GNSS + dead reckoning */
    GPS_FIX_TIME = 5,     /* Time only */
} gps_fix_t;

/* ══════════════════════════════════════════════════════
 *  GPS data (NAV-PVT에서 추출)
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    /* Time */
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;

    /* Fix */
    gps_fix_t fix_type;
    uint8_t num_sv; /* 사용 위성 수 */

    /* Position */
    int32_t lat;     /* 위도 (1e-7 deg) */
    int32_t lon;     /* 경도 (1e-7 deg) */
    int32_t alt_msl; /* 해발 고도 (mm) */

    /* Velocity */
    int32_t vel_n;  /* 북 방향 속도 (mm/s) */
    int32_t vel_e;  /* 동 방향 속도 (mm/s) */
    int32_t vel_d;  /* 하 방향 속도 (mm/s) */
    uint32_t speed; /* 지면 속도 (mm/s) */

    /* Accuracy estimates */
    uint32_t h_acc; /* 수평 정확도 (mm) */
    uint32_t v_acc; /* 수직 정확도 (mm) */
} gps_data_t;

/* ══════════════════════════════════════════════════════
 *  UBX parser state machine
 * ══════════════════════════════════════════════════════ */

typedef enum
{
    UBX_STATE_SYNC1,
    UBX_STATE_SYNC2,
    UBX_STATE_CLASS,
    UBX_STATE_ID,
    UBX_STATE_LEN_L,
    UBX_STATE_LEN_H,
    UBX_STATE_PAYLOAD,
    UBX_STATE_CK_A,
    UBX_STATE_CK_B,
} ubx_state_t;

typedef struct
{
    uart_dev_t *uart_dev;

    /* Parser state */
    ubx_state_t state;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t payload_len;
    uint16_t payload_idx;
    uint8_t payload[UBX_MAX_PAYLOAD];
    uint8_t ck_a;
    uint8_t ck_b;

    /* Latest GPS data */
    gps_data_t data;
    uint8_t data_valid; /* NAV-PVT를 최소 1회 성공적으로 파싱 */
} ubx_dev_t;

/* ══════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════ */

/**
 * @brief Initialize u-blox M10 GPS
 * @param dev  UBX device descriptor
 * @return 0 on success, negative error code on failure
 *
 * NAV-PVT 메시지를 1Hz로 출력하도록 설정.
 */
int ubx_init(ubx_dev_t *dev);

/**
 * @brief Feed one byte to the UBX parser
 * @param dev   UBX device descriptor
 * @param byte  Received byte from UART
 * @return 1 if a complete NAV-PVT message was parsed, 0 otherwise
 *
 * UART 수신 콜백 또는 폴링 루프에서 바이트마다 호출.
 */
int ubx_parse_byte(ubx_dev_t *dev, uint8_t byte);

/**
 * @brief Poll and parse incoming UART data
 * @param dev  UBX device descriptor
 * @return 1 if new GPS data available, 0 if no new data, negative on error
 *
 * 내부적으로 바이트를 읽어서 ubx_parse_byte에 전달.
 * 새 NAV-PVT가 파싱되면 1 반환.
 */
int ubx_poll(ubx_dev_t *dev);

/**
 * @brief Get latest GPS data
 * @param dev   UBX device descriptor
 * @param data  Output: GPS data
 * @return 0 on success, SENSOR_ERR_TIMEOUT if no valid data yet
 */
int ubx_get_data(const ubx_dev_t *dev, gps_data_t *data);

#endif /* UBX_M10_H */
