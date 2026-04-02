/**
 * @file watchdog.c
 * @brief Watchdog timer implementation
 *
 * IWDG (Independent Watchdog):
 *   Driven by LSI (32kHz), independent of main clock.
 *   MCU resets if not refreshed within timeout.
 */

#include "watchdog.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

/* ── IWDG handle ─────────────────────────────────────── */

static IWDG_HandleTypeDef hiwdg;

/* ── Task health flags ────────────────────────────── */

static volatile uint8_t task_alive[WATCHDOG_TASK_COUNT];

/* ── Public API ─────────────────────────────────────── */

void watchdog_init(void)
{
    /* Reset all flags */
    for (int i = 0; i < WATCHDOG_TASK_COUNT; i++)
    {
        task_alive[i] = 0;
    }

    /*
     * IWDG Configuration:
     * LSI = 32kHz
     * Prescaler = 32 → IWDG clock = 32000/32 = 1000 Hz
     * Reload = 500 → timeout = 500ms
     *
     * Monitor task refreshes every 100ms (10Hz), so
     * 500ms timeout provides sufficient margin.
     */
    hiwdg.Instance = IWDG1;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Reload = 500;
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;

    HAL_IWDG_Init(&hiwdg);
}

void watchdog_feed(watchdog_task_id_t task_id)
{
    if (task_id < WATCHDOG_TASK_COUNT)
    {
        task_alive[task_id] = 1;
    }
}

void watchdog_task_func(void *param)
{
    (void)param;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        /*
         * Only critical tasks (sensor, ahrs, control) checked for reset.
         * Optional tasks (mavlink, gps, log) do not trigger reset.
         * GPS not getting fix indoors is normal; logging failure should not reset.
         */
        uint8_t critical_alive = 1;
        for (int i = 0; i < WATCHDOG_CRITICAL_COUNT; i++)
        {
            if (!task_alive[i])
            {
                critical_alive = 0;
                break;
            }
        }

        if (critical_alive)
        {
            HAL_IWDG_Refresh(&hiwdg);
        }

        /* Reset all flags (both critical and optional) */
        for (int i = 0; i < WATCHDOG_TASK_COUNT; i++)
        {
            task_alive[i] = 0;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));  /* 10Hz */
    }
}
