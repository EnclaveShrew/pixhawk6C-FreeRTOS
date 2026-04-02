/**
 * @file sd_logger.c
 * @brief SD card logging implementation
 *
 * Collects log data with double buffering and writes to SD.
 * flush called from log task (low priority).
 *
 * TODO: FatFS integration (interface only for now)
 */

#include "sd_logger.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* ── Write buffer ─────────────────────────────────────── */

#define LOG_BUF_SIZE    4096    /* 4KB buffer */

static uint8_t log_buffer[LOG_BUF_SIZE];
static uint16_t log_buf_pos = 0;
static uint8_t log_initialized = 0;

/* TODO: FatFS file handle */
/* static FIL log_file; */

/* ── Public API ─────────────────────────────────────── */

int sd_logger_init(void)
{
    log_buf_pos = 0;
    log_initialized = 0;

    /*
     * TODO: FatFS implementation
     * 1. f_mount() — Mount SD card
     * 2. Scan existing log file numbers (LOG_0001.bin ~ LOG_9999.bin)
     * 3. f_open() next number -- create new log file
     */

    log_initialized = 1;
    return 0;
}

int sd_logger_write(log_type_t type, const void *payload, uint8_t len)
{
    if (!log_initialized)
    {
        return -1;
    }

    uint16_t total = sizeof(log_header_t) + len;

    /* Buffer overflow prevention: flush when space is insufficient */
    if (log_buf_pos + total > LOG_BUF_SIZE)
    {
        if (sd_logger_flush() < 0)
        {
            return -1;
        }
    }

    /* Write header */
    log_header_t header;
    header.timestamp_ms = HAL_GetTick();
    header.type = (uint8_t)type;
    header.len = len;

    memcpy(&log_buffer[log_buf_pos], &header, sizeof(log_header_t));
    log_buf_pos += sizeof(log_header_t);

    /* Write payload */
    memcpy(&log_buffer[log_buf_pos], payload, len);
    log_buf_pos += len;

    return 0;
}

int sd_logger_flush(void)
{
    if (!log_initialized || log_buf_pos == 0)
    {
        return 0;
    }

    /*
     * TODO: FatFS implementation
     * UINT bw;
     * FRESULT res = f_write(&log_file, log_buffer, log_buf_pos, &bw);
     * if (res != FR_OK || bw != log_buf_pos) return -1;
     * f_sync(&log_file);
     */

    log_buf_pos = 0;
    return 0;
}

void sd_logger_close(void)
{
    sd_logger_flush();

    /*
     * TODO: FatFS implementation
     * f_close(&log_file);
     * f_mount(NULL, "", 0);
     */

    log_initialized = 0;
}
