/**
 * @file sd_logger.h
 * @brief SD card logging system
 *
 * Records flight data to SD card in binary format.
 * Based on FatFS, called from log task.
 */

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "sensor_types.h"
#include <stdint.h>

/* ══════════════════════════════════════════════════════
 *  Log entry types
 * ══════════════════════════════════════════════════════ */

typedef enum
{
    LOG_TYPE_IMU = 0x01,
    LOG_TYPE_ATTITUDE = 0x02,
    LOG_TYPE_GPS = 0x03,
    LOG_TYPE_CONTROL = 0x04,
    LOG_TYPE_BARO = 0x05,
    LOG_TYPE_SYSTEM = 0x06,
} log_type_t;

/* ══════════════════════════════════════════════════════
 *  Log entry header (Common to all entries)
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    uint32_t timestamp_ms;  /* Elapsed time since boot */
    uint8_t type;           /* log_type_t */
    uint8_t len;            /* Payload length */
} __attribute__((packed)) log_header_t;

/* ══════════════════════════════════════════════════════
 *  Log payloads
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    float accel_x, accel_y, accel_z;    /* m/s² */
    float gyro_x, gyro_y, gyro_z;      /* rad/s */
} __attribute__((packed)) log_imu_t;

typedef struct
{
    float roll, pitch, yaw;             /* rad */
} __attribute__((packed)) log_attitude_t;

typedef struct
{
    int32_t lat, lon;                   /* 1e-7 deg */
    int32_t alt_msl;                    /* mm */
    uint8_t fix_type;
    uint8_t num_sv;
} __attribute__((packed)) log_gps_t;

typedef struct
{
    float roll_out, pitch_out, yaw_out; /* -1~+1 */
    float thrust;                       /* 0~1 */
    uint8_t ctrl_type;
} __attribute__((packed)) log_control_t;

typedef struct
{
    float temperature;                  /* °C */
    float pressure;                     /* Pa */
    float altitude;                     /* m (barometric) */
} __attribute__((packed)) log_baro_t;

/* ══════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════ */

/**
 * @brief Initialize SD card and create log file
 * @return 0 on success, negative on error
 *
 * Filename: LOG_XXXX.bin (auto-incrementing number)
 */
int sd_logger_init(void);

/**
 * @brief Write a log entry
 * @param type     Log entry type
 * @param payload  Payload data
 * @param len      Payload length
 * @return 0 on success, negative on error
 *
 * header(6 bytes) + payload written to buffer.
 * flush to SD card when buffer is full.
 */
int sd_logger_write(log_type_t type, const void *payload, uint8_t len);

/**
 * @brief Flush write buffer to SD card
 * @return 0 on success, negative on error
 */
int sd_logger_flush(void);

/**
 * @brief Close log file and unmount SD card
 */
void sd_logger_close(void);

#endif /* SD_LOGGER_H */
