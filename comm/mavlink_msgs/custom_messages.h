/**
 * @file custom_messages.h
 * @brief Custom MAVLink message definitions
 *
 * 표준 MAVLink에 없는 프로젝트 전용 메시지 정의.
 * GCS(QGC 등)에서 인식하려면 같은 XML로 빌드 필요.
 *
 * ID 범위: 150000~ (개발용)
 */

#ifndef CUSTOM_MESSAGES_H
#define CUSTOM_MESSAGES_H

#include <stdint.h>

/* ══════════════════════════════════════════════════════
 *  CONTROLLER_SWITCH (ID: 150000)
 *  GCS → FC: 제어기 전환 명령
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
 *  FC → GCS: 현재 제어기 상태 보고
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
