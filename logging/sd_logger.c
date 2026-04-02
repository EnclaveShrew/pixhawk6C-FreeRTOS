/**
 * @file sd_logger.c
 * @brief SD card logging implementation
 *
 * 더블 버퍼링으로 로그 데이터를 모아서 SD에 기록.
 * 로그 태스크(저우선순위)에서 flush 호출.
 *
 * TODO: FatFS 연동 (현재는 인터페이스만 정의)
 */

#include "sd_logger.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* ── 쓰기 버퍼 ─────────────────────────────────────── */

#define LOG_BUF_SIZE    4096    /* 4KB 버퍼 */

static uint8_t log_buffer[LOG_BUF_SIZE];
static uint16_t log_buf_pos = 0;
static uint8_t log_initialized = 0;

/* TODO: FatFS 파일 핸들 */
/* static FIL log_file; */

/* ── Public API ─────────────────────────────────────── */

int sd_logger_init(void)
{
    log_buf_pos = 0;
    log_initialized = 0;

    /*
     * TODO: FatFS 구현
     * 1. f_mount() — SD카드 마운트
     * 2. 기존 로그 파일 번호 스캔 (LOG_0001.bin ~ LOG_9999.bin)
     * 3. 다음 번호로 f_open() — 새 로그 파일 생성
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

    /* 버퍼 오버플로우 방지: 공간 부족하면 flush */
    if (log_buf_pos + total > LOG_BUF_SIZE)
    {
        if (sd_logger_flush() < 0)
        {
            return -1;
        }
    }

    /* 헤더 기록 */
    log_header_t header;
    header.timestamp_ms = HAL_GetTick();
    header.type = (uint8_t)type;
    header.len = len;

    memcpy(&log_buffer[log_buf_pos], &header, sizeof(log_header_t));
    log_buf_pos += sizeof(log_header_t);

    /* 페이로드 기록 */
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
     * TODO: FatFS 구현
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
     * TODO: FatFS 구현
     * f_close(&log_file);
     * f_mount(NULL, "", 0);
     */

    log_initialized = 0;
}
