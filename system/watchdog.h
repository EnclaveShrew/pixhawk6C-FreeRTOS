/**
 * @file watchdog.h
 * @brief Watchdog timer — task health monitoring
 *
 * 각 중요 태스크가 주기적으로 feed()를 호출하여 "살아있음"을 보고.
 * 감시 태스크가 모든 플래그를 확인한 뒤에만 IWDG를 리프레시.
 * 어떤 태스크가 무한루프/교착에 빠지면 리프레시가 안 되어 MCU 리셋.
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>

/* ══════════════════════════════════════════════════════
 *  Task IDs for watchdog feed
 * ══════════════════════════════════════════════════════ */

typedef enum
{
    WATCHDOG_TASK_SENSOR = 0,
    WATCHDOG_TASK_AHRS,
    WATCHDOG_TASK_CONTROL,
    WATCHDOG_TASK_MAVLINK,
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
 * @brief Report task alive (각 태스크에서 주기적으로 호출)
 * @param task_id  Task identifier
 */
void watchdog_feed(watchdog_task_id_t task_id);

/**
 * @brief Watchdog monitor task function (FreeRTOS task)
 *
 * 모든 태스크의 feed 플래그를 확인하고,
 * 전부 살아있으면 IWDG 리프레시.
 * 10Hz로 실행.
 */
void watchdog_task_func(void *param);

#endif /* WATCHDOG_H */
