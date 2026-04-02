/**
 * @file stm32f103_io.h
 * @brief IO Processor (STM32F103) communication and PWM management
 *
 * FMU(H743) ↔ IO(F103) 통신 프로토콜:
 *   FMU → IO: PWM 출력 값 (최대 8ch), 설정 명령
 *   IO → FMU: RC 입력 값, IO 상태
 *   주기: ~400Hz (PWM 업데이트 주기)
 *   체크섬: CRC-8
 *
 * IO Processor 페일세이프:
 *   FMU 비응답 감지 시 → RC 패스스루 또는 안전 스로틀
 */

#ifndef STM32F103_IO_H
#define STM32F103_IO_H

#include "uart_wrapper.h"
#include <stdint.h>

/* ══════════════════════════════════════════════════════
 *  Protocol constants
 * ══════════════════════════════════════════════════════ */

#define IO_SYNC_BYTE        0xA5
#define IO_MAX_PWM_CHANNELS 8
#define IO_MAX_RC_CHANNELS  16

/* Packet types */
#define IO_PKT_PWM_OUTPUT   0x01    /* FMU → IO: PWM 값 전달 */
#define IO_PKT_RC_INPUT     0x02    /* IO → FMU: RC 입력 */
#define IO_PKT_STATUS       0x03    /* IO → FMU: 상태 보고 */
#define IO_PKT_CONFIG       0x04    /* FMU → IO: 설정 변경 */
#define IO_PKT_FAILSAFE     0x05    /* FMU → IO: 페일세이프 값 설정 */

/* IO status flags */
#define IO_STATUS_OK        0x00
#define IO_STATUS_RC_LOST   0x01
#define IO_STATUS_FMU_LOST  0x02

/* ══════════════════════════════════════════════════════
 *  Data structures
 * ══════════════════════════════════════════════════════ */

/* FMU → IO: PWM 출력 (18 bytes) */
typedef struct
{
    uint16_t pwm[IO_MAX_PWM_CHANNELS];  /* PWM 값 (1000~2000 us) */
    uint8_t num_channels;
    uint8_t armed;                       /* 0=disarmed, 1=armed */
} __attribute__((packed)) io_pwm_packet_t;

/* IO → FMU: RC 입력 */
typedef struct
{
    uint16_t rc[IO_MAX_RC_CHANNELS];    /* RC 채널 값 (1000~2000 us) */
    uint8_t num_channels;
    uint8_t rssi;                        /* 신호 강도 (0~255) */
    uint8_t status;                      /* IO 상태 플래그 */
} __attribute__((packed)) io_rc_packet_t;

/* IO Processor 상태 */
typedef struct
{
    uart_dev_t *uart_dev;

    /* 최신 RC 입력 */
    io_rc_packet_t rc_input;
    uint8_t rc_valid;

    /* 페일세이프 PWM 값 */
    uint16_t failsafe_pwm[IO_MAX_PWM_CHANNELS];

    /* 통신 상태 */
    uint32_t last_rx_time;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t error_count;
} io_processor_t;

/* ══════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════ */

/**
 * @brief Initialize IO Processor communication
 * @param io        IO processor state
 * @param uart_dev  UART device for FMU ↔ IO
 * @return 0 on success
 */
int io_init(io_processor_t *io, uart_dev_t *uart_dev);

/**
 * @brief Send PWM output values to IO Processor
 * @param io   IO processor state
 * @param pwm  PWM values array (1000~2000 us)
 * @param num  Number of channels (1~8)
 * @param armed  Arm state
 * @return 0 on success
 */
int io_send_pwm(io_processor_t *io, const uint16_t *pwm, uint8_t num, uint8_t armed);

/**
 * @brief Set failsafe PWM values (FMU 비응답 시 IO가 사용)
 * @param io   IO processor state
 * @param pwm  Failsafe PWM values
 * @param num  Number of channels
 * @return 0 on success
 */
int io_set_failsafe(io_processor_t *io, const uint16_t *pwm, uint8_t num);

/**
 * @brief Poll for incoming RC/status data from IO Processor
 * @param io  IO processor state
 * @return 1 if new RC data, 0 if none, negative on error
 */
int io_poll(io_processor_t *io);

/**
 * @brief Get latest RC input
 * @param io  IO processor state
 * @param rc  Output: RC packet
 * @return 0 on success, -1 if no valid data
 */
int io_get_rc(const io_processor_t *io, io_rc_packet_t *rc);

/**
 * @brief Check if IO Processor communication is healthy
 * @param io  IO processor state
 * @return 1 if healthy, 0 if timeout
 */
int io_is_healthy(const io_processor_t *io);

#endif /* STM32F103_IO_H */
