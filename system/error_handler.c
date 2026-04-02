/**
 * @file error_handler.c
 * @brief System error handling and failsafe implementation
 */

#include "error_handler.h"
#include "stm32h7xx_hal.h"

/* ── 에러 카운터 ───────────────────────────────────── */

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

    /* TODO: SD 로그 기록 */
    /* TODO: MAVLink 경고 전송 */
}

void error_handler_fatal(system_error_t err)
{
    /* 인터럽트 비활성화 */
    __disable_irq();

    last_error = err;

    /* LED 점멸로 에러 표시 (에러 코드만큼 깜박) */
    for (;;)
    {
        for (int i = 0; i < (int)err; i++)
        {
            /* TODO: LED ON */
            for (volatile int d = 0; d < 500000; d++);
            /* TODO: LED OFF */
            for (volatile int d = 0; d < 500000; d++);
        }
        /* 긴 대기 후 반복 */
        for (volatile int d = 0; d < 3000000; d++);
    }
}

failsafe_action_t error_handler_get_failsafe(void)
{
    /* 양쪽 IMU 모두 실패 → 비상 착륙 */
    if (error_counts[ERR_IMU_BOTH_FAIL] > 0)
    {
        return FAILSAFE_EMERGENCY_LAND;
    }

    /* Primary IMU만 실패 → 이중 IMU 전환 */
    if (error_counts[ERR_IMU_PRIMARY_FAIL] > 3)
    {
        return FAILSAFE_IMU_SWITCH;
    }

    /* RC 신호 로스트 → RTL */
    if (error_counts[ERR_RC_LOST] > 10)
    {
        return FAILSAFE_RETURN_TO_LAUNCH;
    }

    /* GPS 실패 → 고도 유지 */
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
