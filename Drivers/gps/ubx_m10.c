/**
 * @file ubx_m10.c
 * @brief u-blox M10 GPS driver — UBX protocol parser implementation
 *
 * UBX 패킷 구조:
 * [SYNC1 0xB5] [SYNC2 0x62] [CLASS] [ID] [LEN_L] [LEN_H] [PAYLOAD...] [CK_A] [CK_B]
 *
 * 체크섬: CLASS부터 PAYLOAD 끝까지 Fletcher-8 알고리즘
 */

#include "ubx_m10.h"
#include <string.h>

/* ── UBX 메시지 빌더 (설정용) ──────────────────────── */

/**
 * @brief UBX 설정 메시지 전송
 */
static int ubx_send_msg(ubx_dev_t *dev, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    /* Header: SYNC1 + SYNC2 + CLASS + ID + LEN_L + LEN_H */
    uint8_t header[6] = {UBX_SYNC1, UBX_SYNC2, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};

    /* 체크섬 계산: CLASS부터 PAYLOAD 끝까지 */
    uint8_t ck_a = 0, ck_b = 0;

    /* header[2..5] = CLASS, ID, LEN_L, LEN_H */
    for (int i = 2; i < 6; i++)
    {
        ck_a += header[i];
        ck_b += ck_a;
    }
    for (uint16_t i = 0; i < len; i++)
    {
        ck_a += payload[i];
        ck_b += ck_a;
    }

    /* 전송: header → payload → checksum */
    if (uart_send(dev->uart_dev, header, 6) < 0)
    {
        return -1;
    }
    if (len > 0 && uart_send(dev->uart_dev, payload, len) < 0)
    {
        return -1;
    }

    uint8_t ck[2] = {ck_a, ck_b};
    return uart_send(dev->uart_dev, ck, 2);
}

/* ── NAV-PVT 페이로드 파싱 ─────────────────────────── */

/**
 * @brief 리틀 엔디언 바이트를 정수로 변환
 */
static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t le32_signed(const uint8_t *p)
{
    return (int32_t)le32(p);
}

static void ubx_parse_nav_pvt(ubx_dev_t *dev)
{
    const uint8_t *p = dev->payload;

    /* NAV-PVT 페이로드 오프셋 (데이터시트 기준) */
    dev->data.year = le16(&p[4]);
    dev->data.month = p[6];
    dev->data.day = p[7];
    dev->data.hour = p[8];
    dev->data.min = p[9];
    dev->data.sec = p[10];

    dev->data.fix_type = (gps_fix_t)p[20];
    dev->data.num_sv = p[23];

    dev->data.lon = le32_signed(&p[24]);     /* 1e-7 deg */
    dev->data.lat = le32_signed(&p[28]);     /* 1e-7 deg */
    dev->data.alt_msl = le32_signed(&p[36]); /* mm */

    dev->data.vel_n = le32_signed(&p[48]); /* mm/s */
    dev->data.vel_e = le32_signed(&p[52]); /* mm/s */
    dev->data.vel_d = le32_signed(&p[56]); /* mm/s */
    dev->data.speed = le32(&p[60]);        /* mm/s */

    dev->data.h_acc = le32(&p[40]); /* mm */
    dev->data.v_acc = le32(&p[44]); /* mm */

    dev->data_valid = 1;
}

/* ── 초기화 ─────────────────────────────────────────── */

int ubx_init(ubx_dev_t *dev)
{
    /* Parser 상태 초기화 */
    dev->state = UBX_STATE_SYNC1;
    dev->data_valid = 0;
    memset(&dev->data, 0, sizeof(gps_data_t));

    /*
     * CFG-MSG: NAV-PVT를 매 네비게이션 주기마다 출력
     * payload: [CLASS] [ID] [rate]
     */
    uint8_t cfg_nav_pvt[] = {UBX_CLASS_NAV, UBX_NAV_PVT, 0x01};
    if (ubx_send_msg(dev, UBX_CLASS_CFG, UBX_CFG_MSG, cfg_nav_pvt, sizeof(cfg_nav_pvt)) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    return SENSOR_OK;
}

/* ── 바이트 단위 파서 (상태 머신) ──────────────────── */

int ubx_parse_byte(ubx_dev_t *dev, uint8_t byte)
{
    switch (dev->state)
    {
    case UBX_STATE_SYNC1:
        if (byte == UBX_SYNC1)
        {
            dev->state = UBX_STATE_SYNC2;
        }
        break;

    case UBX_STATE_SYNC2:
        if (byte == UBX_SYNC2)
        {
            dev->state = UBX_STATE_CLASS;
            dev->ck_a = 0;
            dev->ck_b = 0;
        }
        else
        {
            dev->state = UBX_STATE_SYNC1;
        }
        break;

    case UBX_STATE_CLASS:
        dev->msg_class = byte;
        dev->ck_a += byte;
        dev->ck_b += dev->ck_a;
        dev->state = UBX_STATE_ID;
        break;

    case UBX_STATE_ID:
        dev->msg_id = byte;
        dev->ck_a += byte;
        dev->ck_b += dev->ck_a;
        dev->state = UBX_STATE_LEN_L;
        break;

    case UBX_STATE_LEN_L:
        dev->payload_len = byte;
        dev->ck_a += byte;
        dev->ck_b += dev->ck_a;
        dev->state = UBX_STATE_LEN_H;
        break;

    case UBX_STATE_LEN_H:
        dev->payload_len |= ((uint16_t)byte << 8);
        dev->ck_a += byte;
        dev->ck_b += dev->ck_a;
        dev->payload_idx = 0;

        if (dev->payload_len > UBX_MAX_PAYLOAD)
        {
            /* 페이로드가 버퍼보다 크면 무시 */
            dev->state = UBX_STATE_SYNC1;
        }
        else if (dev->payload_len == 0)
        {
            dev->state = UBX_STATE_CK_A;
        }
        else
        {
            dev->state = UBX_STATE_PAYLOAD;
        }
        break;

    case UBX_STATE_PAYLOAD:
        dev->payload[dev->payload_idx++] = byte;
        dev->ck_a += byte;
        dev->ck_b += dev->ck_a;

        if (dev->payload_idx >= dev->payload_len)
        {
            dev->state = UBX_STATE_CK_A;
        }
        break;

    case UBX_STATE_CK_A:
        if (byte == dev->ck_a)
        {
            dev->state = UBX_STATE_CK_B;
        }
        else
        {
            /* 체크섬 불일치 */
            dev->state = UBX_STATE_SYNC1;
        }
        break;

    case UBX_STATE_CK_B:
        dev->state = UBX_STATE_SYNC1;

        if (byte == dev->ck_b)
        {
            /* 체크섬 통과 — 메시지 처리 */
            if (dev->msg_class == UBX_CLASS_NAV && dev->msg_id == UBX_NAV_PVT && dev->payload_len == UBX_NAV_PVT_LEN)
            {
                ubx_parse_nav_pvt(dev);
                return 1;
            }
        }
        break;
    }

    return 0;
}

/* ── 폴링 방식 수신 + 파싱 ─────────────────────────── */

int ubx_poll(ubx_dev_t *dev)
{
    uint8_t byte;

    while (uart_receive_byte(dev->uart_dev, &byte) == 0)
    {
        if (ubx_parse_byte(dev, byte) == 1)
        {
            return 1; /* 새 NAV-PVT 데이터 */
        }
    }

    return 0;
}

/* ── GPS 데이터 가져오기 ───────────────────────────── */

int ubx_get_data(const ubx_dev_t *dev, gps_data_t *data)
{
    if (!dev->data_valid)
    {
        return SENSOR_ERR_TIMEOUT;
    }

    *data = dev->data;
    return SENSOR_OK;
}
