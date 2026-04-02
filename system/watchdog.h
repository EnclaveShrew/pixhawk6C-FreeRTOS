/**
 * @file watchdog.h
 * @brief Watchdog timer — task health monitoring
 *
 * Each critical task periodically calls feed() to report it is alive.
 * Monitor task refreshes IWDG only after checking all flags.
 * If any task deadlocks, no refresh -> MCU reset.
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>

/* ══════════════════════════════════════════════════════
 *  Task IDs for watchdog feed
 * ══════════════════════════════════════════════════════ */

typedef enum
{
    /* Critical: feed failure -> watchdog reset */
    WATCHDOG_TASK_SENSOR = 0,
    WATCHDOG_TASK_AHRS,
    WATCHDOG_TASK_CONTROL,
    WATCHDOG_CRITICAL_COUNT,    /* up to here are critical */

    /* Optional: feed failure does not reset (error report only) */
    WATCHDOG_TASK_MAVLINK = WATCHDOG_CRITICAL_COUNT,
    WATCHDOG_TASK_GPS,
    WATCHDOG_TASK_LOG,
    WATCHDOG_TASK_COUNT,
} watchdog_task_id_t;

/* ══════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════ */

/**
 * @brief Initialize IWDG and watchdog monitoring
 */
void watchdog_init(void);

/**
 * @brief Report task alive (Called periodically from each task)
 * @param task_id  Task identifier
 */
void watchdog_feed(watchdog_task_id_t task_id);

/**
 * @brief Watchdog monitor task function (FreeRTOS task)
 *
 * Checks all task feed flags,
 * refreshes IWDG if all alive.
 * Runs at 10Hz.
 */
void watchdog_task_func(void *param);

#endif /* WATCHDOG_H */
