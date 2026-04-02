/**
 * @file stm32f103_io.c
 * @brief IO Processor communication implementation
 *
 * Packet structure:
 *   [SYNC 0xA5] [TYPE] [LEN] [PAYLOAD...] [CRC8]
 */

#include "stm32f103_io.h"
#include "stm32h7xx_hal.h"
#include <string.h>

#define IO_COMM_TIMEOUT_MS  200     /* IO non-response timeout */

/* ── CRC-8 (simple checksum) ─────────────────────────── */

static uint8_t crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* ── Internal: send packet ──────────────────────────────── */

static int io_send_packet(io_processor_t *io, uint8_t type,
                          const uint8_t *payload, uint8_t len)
{
    /* Header: SYNC + TYPE + LEN */
    uint8_t header[3] = { IO_SYNC_BYTE, type, len };

    if (uart_send(io->uart_dev, header, 3) < 0)
    {
        return -1;
    }
    if (len > 0 && uart_send(io->uart_dev, payload, len) < 0)
    {
        return -1;
    }

    /* CRC: TYPE + LEN + PAYLOAD */
    uint8_t crc_data[2] = { type, len };
    uint8_t crc = crc8(crc_data, 2);
    if (len > 0)
    {
        uint8_t payload_crc = crc8(payload, len);
        crc ^= payload_crc;
    }

    if (uart_send(io->uart_dev, &crc, 1) < 0)
    {
        return -1;
    }

    io->tx_count++;
    return 0;
}

/* ── Public API ─────────────────────────────────────── */

int io_init(io_processor_t *io, uart_dev_t *uart_dev)
{
    memset(io, 0, sizeof(io_processor_t));
    io->uart_dev = uart_dev;

    /* Default failsafe: all channels 1000us (minimum) */
    for (int i = 0; i < IO_MAX_PWM_CHANNELS; i++)
    {
        io->failsafe_pwm[i] = 1000;
    }

    return 0;
}

int io_send_pwm(io_processor_t *io, const uint16_t *pwm, uint8_t num, uint8_t armed)
{
    io_pwm_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.num_channels = (num > IO_MAX_PWM_CHANNELS) ? IO_MAX_PWM_CHANNELS : num;
    pkt.armed = armed;

    for (int i = 0; i < pkt.num_channels; i++)
    {
        pkt.pwm[i] = pwm[i];
    }

    return io_send_packet(io, IO_PKT_PWM_OUTPUT, (const uint8_t *)&pkt, sizeof(pkt));
}

int io_set_failsafe(io_processor_t *io, const uint16_t *pwm, uint8_t num)
{
    uint8_t count = (num > IO_MAX_PWM_CHANNELS) ? IO_MAX_PWM_CHANNELS : num;

    for (int i = 0; i < count; i++)
    {
        io->failsafe_pwm[i] = pwm[i];
    }

    /* Send failsafe values to IO */
    io_pwm_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.num_channels = count;
    pkt.armed = 0;
    memcpy(pkt.pwm, io->failsafe_pwm, count * sizeof(uint16_t));

    return io_send_packet(io, IO_PKT_FAILSAFE, (const uint8_t *)&pkt, sizeof(pkt));
}

int io_poll(io_processor_t *io)
{
    /* TODO: Byte-level state machine parser (same pattern as UBX/MAVLink) */
    /*
     * SYNC(0xA5) -> TYPE -> LEN -> PAYLOAD -> CRC verify
     * TYPE == IO_PKT_RC_INPUT -> update io->rc_input
     * TYPE == IO_PKT_STATUS -> handle status
     */

    return 0;
}

int io_get_rc(const io_processor_t *io, io_rc_packet_t *rc)
{
    if (!io->rc_valid)
    {
        return -1;
    }
    *rc = io->rc_input;
    return 0;
}

int io_is_healthy(const io_processor_t *io)
{
    uint32_t now = HAL_GetTick();
    return (now - io->last_rx_time) < IO_COMM_TIMEOUT_MS;
}
