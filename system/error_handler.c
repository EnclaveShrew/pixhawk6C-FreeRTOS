/**
 * @file error_handler.c
 * @brief System error handling and failsafe implementation
 */

#include "error_handler.h"
#include "stm32h7xx_hal.h"

/* ── Error counter ───────────────────────────────────── */

#define ERROR_TYPE_COUNT    12

static volatile uint32_t error_counts[ERROR_TYPE_COUNT];
static volatile system_error_t last_error = ERR_NONE;

/* ── Public API ─────────────────────────────────────── */

void error_handler_report(system_error_t err)
{
    if (err > 0 && err < ERROR_TYPE_COUNT)
    {
        error_counts[err]++;
        last_error = err;
    }

    /* TODO: SD card logging */
    /* TODO: MAVLink warning */
}

void error_handler_fatal(system_error_t err)
{
    /*
     * Disable interrupts to stop FreeRTOS scheduler and all tasks.
     * IWDG is a hardware timer independent of interrupts —
     * it will reset MCU after timeout (~500ms) since we don't refresh.
     *
     * This gives a brief window to indicate the error via LED,
     * then the watchdog resets the system for recovery.
     */
    __disable_irq();

    last_error = err;

    /* Brief LED indication before watchdog reset */
    /* TODO: LED ON */
    for (volatile int d = 0; d < 1000000; d++);
    /* TODO: LED OFF */

    /* Wait for IWDG reset (no refresh -> reset within 500ms) */
    for (;;);
}

failsafe_action_t error_handler_get_failsafe(void)
{
    /* Both IMUs failed -> emergency landing */
    if (error_counts[ERR_IMU_BOTH_FAIL] > 0)
    {
        return FAILSAFE_EMERGENCY_LAND;
    }

    /* Primary IMU failed -> switch to redundant IMU */
    if (error_counts[ERR_IMU_PRIMARY_FAIL] > 3)
    {
        return FAILSAFE_IMU_SWITCH;
    }

    /* RC signal lost -> RTL */
    if (error_counts[ERR_RC_LOST] > 10)
    {
        return FAILSAFE_RETURN_TO_LAUNCH;
    }

    /* GPS failure -> altitude hold */
    if (error_counts[ERR_GPS_FAIL] > 5)
    {
        return FAILSAFE_ALTITUDE_HOLD;
    }

    return FAILSAFE_NONE;
}

uint32_t error_handler_get_count(system_error_t err)
{
    if (err > 0 && err < ERROR_TYPE_COUNT)
    {
        return error_counts[err];
    }
    return 0;
}
