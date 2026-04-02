/**
 * @file watchdog.c
 * @brief Watchdog timer implementation
 *
 * IWDG (Independent Watchdog):
 *   LSI 클럭(32kHz)으로 구동, 메인 클럭과 독립.
 *   타임아웃 내에 리프레시하지 않으면 MCU 리셋.
 */

#include "watchdog.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

/* ── IWDG 핸들 ─────────────────────────────────────── */

static IWDG_HandleTypeDef hiwdg;

/* ── 태스크 헬스 플래그 ────────────────────────────── */

static volatile uint8_t task_alive[WATCHDOG_TASK_COUNT];

/* ── Public API ─────────────────────────────────────── */

void watchdog_init(void)
{
    /* 모든 플래그 초기화 */
    for (int i = 0; i < WATCHDOG_TASK_COUNT; i++)
    {
        task_alive[i] = 0;
    }

    /*
     * IWDG 설정:
     * LSI = 32kHz
     * Prescaler = 32 → IWDG 클럭 = 32000/32 = 1000 Hz
     * Reload = 500 → 타임아웃 = 500ms
     *
     * 감시 태스크가 100ms(10Hz)마다 리프레시하므로
     * 500ms 타임아웃은 충분한 여유.
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
        /* 모든 태스크가 살아있는지 확인 */
        uint8_t all_alive = 1;
        for (int i = 0; i < WATCHDOG_TASK_COUNT; i++)
        {
            if (!task_alive[i])
            {
                all_alive = 0;
                break;
            }
        }

        if (all_alive)
        {
            /* 모두 살아있으면 IWDG 리프레시 */
            HAL_IWDG_Refresh(&hiwdg);

            /* 플래그 초기화 (다음 주기에 다시 확인) */
            for (int i = 0; i < WATCHDOG_TASK_COUNT; i++)
            {
                task_alive[i] = 0;
            }
        }
        /* all_alive == 0이면 리프레시 안 함 → 타임아웃 시 리셋 */

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));  /* 10Hz */
    }
}
