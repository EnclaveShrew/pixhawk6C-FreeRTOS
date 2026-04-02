/**
 * @file error_handler.h
 * @brief System error handling and failsafe logic
 */

#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

#include <stdint.h>

/* ══════════════════════════════════════════════════════
 *  Error codes
 * ══════════════════════════════════════════════════════ */

typedef enum
{
    ERR_NONE = 0,

    /* Sensor errors */
    ERR_IMU_PRIMARY_FAIL,
    ERR_IMU_REDUNDANT_FAIL,
    ERR_IMU_BOTH_FAIL,
    ERR_BARO_FAIL,
    ERR_MAG_FAIL,
    ERR_GPS_FAIL,

    /* System errors */
    ERR_SCHEDULER_FAILED,
    ERR_STACK_OVERFLOW,
    ERR_MALLOC_FAILED,

    /* Communication errors */
    ERR_MAVLINK_TIMEOUT,
    ERR_RC_LOST,
} system_error_t;

/* ══════════════════════════════════════════════════════
 *  Failsafe actions
 * ══════════════════════════════════════════════════════ */

typedef enum
{
    FAILSAFE_NONE = 0,
    FAILSAFE_IMU_SWITCH,        /* Switch to redundant IMU */
    FAILSAFE_ALTITUDE_HOLD,     /* Altitude hold (no GPS needed) */
    FAILSAFE_RETURN_TO_LAUNCH,  /* RTL */
    FAILSAFE_EMERGENCY_LAND,    /* Emergency landing */
    FAILSAFE_DISARM,            /* Disarm */
} failsafe_action_t;

/* ══════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════ */

/**
 * @brief Report a non-fatal error (increment error counter, log)
 * @param err  Error code
 */
void error_handler_report(system_error_t err);

/**
 * @brief Handle fatal error (unrecoverable, blink LED then wait for reset)
 * @param err  Error code
 */
void error_handler_fatal(system_error_t err);

/**
 * @brief Determine failsafe action based on current errors
 * @return Recommended failsafe action
 */
failsafe_action_t error_handler_get_failsafe(void);

/**
 * @brief Get error count for a specific error type
 * @param err  Error code
 * @return Number of times this error occurred
 */
uint32_t error_handler_get_count(system_error_t err);

#endif /* ERROR_HANDLER_H */
