/**
 * @file main.c
 * @brief FreeRTOS task initialization and entry point
 *
 * 태스크 구조 및 우선순위:
 *   [7] sensor_task     — 1kHz, IMU 읽기 (최고)
 *   [6] ahrs_task       — 1kHz, 자세 추정
 *   [5] control_task    — 1kHz(rate) / 250Hz(attitude), 제어 루프
 *   [3] mavlink_task    — 50Hz, 텔레메트리 송수신
 *   [2] gps_task        — 10Hz, GPS 파싱
 *   [1] log_task        — 가변, SD카드 로깅 (최저)
 *   [4] watchdog_task   — 10Hz, 태스크 헬스 모니터링
 */

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

/* Sensor drivers */
#include "icm42688p.h"
#include "bmi055.h"
#include "ms5611.h"
#include "ist8310.h"
#include "ubx_m10.h"

/* AHRS */
#include "complementary.h"
#include "ekf.h"

/* Control */
#include "controller_interface.h"
#include "cascade_pid.h"
#include "motor_mixer.h"

/* Communication */
#include "mavlink_handler.h"

/* System */
#include "watchdog.h"
#include "error_handler.h"

/* Logging */
#include "sd_logger.h"

/* ══════════════════════════════════════════════════════
 *  Task priorities (높을수록 높은 우선순위)
 * ══════════════════════════════════════════════════════ */

#define TASK_PRIO_SENSOR    7
#define TASK_PRIO_AHRS      6
#define TASK_PRIO_CONTROL   5
#define TASK_PRIO_WATCHDOG  4
#define TASK_PRIO_MAVLINK   3
#define TASK_PRIO_GPS       2
#define TASK_PRIO_LOG       1

/* ══════════════════════════════════════════════════════
 *  Task stack sizes (words, 1 word = 4 bytes)
 * ══════════════════════════════════════════════════════ */

#define STACK_SENSOR    512
#define STACK_AHRS      1024    /* EKF 행렬 연산으로 큰 스택 필요 */
#define STACK_CONTROL   512
#define STACK_WATCHDOG  256
#define STACK_MAVLINK   512
#define STACK_GPS       512
#define STACK_LOG       512

/* ══════════════════════════════════════════════════════
 *  Task handles
 * ══════════════════════════════════════════════════════ */

static TaskHandle_t sensor_task_handle;
static TaskHandle_t ahrs_task_handle;
static TaskHandle_t control_task_handle;
static TaskHandle_t watchdog_task_handle;
static TaskHandle_t mavlink_task_handle;
static TaskHandle_t gps_task_handle;
static TaskHandle_t log_task_handle;

/* ══════════════════════════════════════════════════════
 *  Shared data (더블 버퍼링 또는 atomic 접근)
 * ══════════════════════════════════════════════════════ */

static volatile vec3f_t g_accel;
static volatile vec3f_t g_gyro;
static volatile float g_roll, g_pitch, g_yaw;

/* Active controller (런타임 교체 가능) */
static controller_t *g_active_ctrl = &cascade_pid_controller;

/* ══════════════════════════════════════════════════════
 *  Sensor task — 1kHz
 * ══════════════════════════════════════════════════════ */

static void sensor_task(void *param)
{
    (void)param;
    TickType_t last_wake = xTaskGetTickCount();

    /* TODO: spi_dev_t, i2c_dev_t 초기화 (CubeMX에서 생성된 핸들 연결) */

    for (;;)
    {
        /* IMU 읽기 (Primary: ICM-42688-P) */
        vec3f_t accel, gyro;
        /* if (icm42688p_read_accel_gyro(&imu_dev, &accel, &gyro) == SENSOR_OK) */
        /* { */
        /*     g_accel = accel; */
        /*     g_gyro = gyro; */
        /* } */

        /* 센서 태스크 헬스 플래그 설정 */
        watchdog_feed(WATCHDOG_TASK_SENSOR);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));  /* 1kHz */
    }
}

/* ══════════════════════════════════════════════════════
 *  AHRS task — 1kHz
 * ══════════════════════════════════════════════════════ */

static void ahrs_task(void *param)
{
    (void)param;
    TickType_t last_wake = xTaskGetTickCount();

    ekf_state_t ekf;
    ekf_init(&ekf, NULL);

    for (;;)
    {
        vec3f_t accel = *(vec3f_t *)&g_accel;
        vec3f_t gyro = *(vec3f_t *)&g_gyro;

        float dt = 0.001f;  /* 1kHz → 1ms */

        ekf_predict(&ekf, &gyro, dt);
        ekf_update_accel(&ekf, &accel);
        /* ekf_update_mag(&ekf, &mag); — 자력계 데이터 있을 때 */

        ekf_get_euler(&ekf, (float *)&g_roll, (float *)&g_pitch, (float *)&g_yaw);

        watchdog_feed(WATCHDOG_TASK_AHRS);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));  /* 1kHz */
    }
}

/* ══════════════════════════════════════════════════════
 *  Control task — 1kHz (rate loop) / 250Hz (attitude loop)
 * ══════════════════════════════════════════════════════ */

static void control_task(void *param)
{
    (void)param;
    TickType_t last_wake = xTaskGetTickCount();

    g_active_ctrl->init(NULL);
    mixer_init(NULL);

    for (;;)
    {
        controller_input_t input;
        controller_output_t output;

        /* 목표 자세 (RC 입력 또는 자동 비행에서 설정) */
        input.target_attitude.w = 1.0f;
        input.target_attitude.x = 0.0f;
        input.target_attitude.y = 0.0f;
        input.target_attitude.z = 0.0f;

        /* 현재 자세 (AHRS 출력) */
        /* TODO: euler_to_quat로 변환하거나 EKF에서 직접 쿼터니언 가져오기 */
        input.current_attitude.w = 1.0f;
        input.current_attitude.x = 0.0f;
        input.current_attitude.y = 0.0f;
        input.current_attitude.z = 0.0f;

        input.gyro = *(vec3f_t *)&g_gyro;
        input.thrust = 0.0f;   /* RC 스로틀에서 설정 */
        input.dt = 0.001f;

        g_active_ctrl->update(&input, &output);

        mixer_output_t motor_out;
        mixer_mix(&output, &motor_out);

        /* TODO: PWM 출력 (IO Processor 또는 직접 타이머) */

        watchdog_feed(WATCHDOG_TASK_CONTROL);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));  /* 1kHz */
    }
}

/* ══════════════════════════════════════════════════════
 *  MAVLink task — 50Hz
 * ══════════════════════════════════════════════════════ */

static mavlink_handler_t g_mav;
static uint32_t g_mav_counter = 0;

static void mavlink_rx_callback(const mavlink_message_t *msg)
{
    /* 수신 메시지 처리 */
    if (msg->msg_id == MAVLINK_MSG_CTRL_SWITCH && msg->payload_len >= 1)
    {
        uint8_t target = msg->payload[0];
        switch (target)
        {
        case CTRL_TYPE_PID:
            g_active_ctrl->reset();
            g_active_ctrl = &cascade_pid_controller;
            g_active_ctrl->init(NULL);
            break;
        case CTRL_TYPE_LQR:
            g_active_ctrl->reset();
            /* g_active_ctrl = &lqr_controller; */
            /* g_active_ctrl->init(NULL); */
            break;
        case CTRL_TYPE_INDI:
            g_active_ctrl->reset();
            /* g_active_ctrl = &indi_controller; */
            /* g_active_ctrl->init(NULL); */
            break;
        }
    }
}

static void mavlink_task(void *param)
{
    (void)param;
    TickType_t last_wake = xTaskGetTickCount();

    /* TODO: UART 핸들 연결 */
    /* mavlink_init(&g_mav, &telem_uart, mavlink_rx_callback); */

    for (;;)
    {
        /* 수신 처리 */
        mavlink_poll(&g_mav);

        /* 송신: Heartbeat 1Hz (50루프마다) */
        if (g_mav_counter % 50 == 0)
        {
            mavlink_send_heartbeat(&g_mav,
                                   MAV_MODE_FLAG_STABILIZE | MAV_MODE_FLAG_CUSTOM,
                                   MAV_STATE_ACTIVE);
            mavlink_send_sys_status(&g_mav, 11100, 75, 0);
        }

        /* 송신: Attitude 10Hz (5루프마다) */
        if (g_mav_counter % 5 == 0)
        {
            mavlink_send_attitude(&g_mav, g_roll, g_pitch, g_yaw, 0, 0, 0);
        }

        g_mav_counter++;

        watchdog_feed(WATCHDOG_TASK_MAVLINK);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));  /* 50Hz */
    }
}

/* ══════════════════════════════════════════════════════
 *  GPS task — 10Hz
 * ══════════════════════════════════════════════════════ */

static void gps_task(void *param)
{
    (void)param;
    TickType_t last_wake = xTaskGetTickCount();

    /* TODO: ubx_dev_t 초기화 */

    for (;;)
    {
        /* ubx_poll(&gps_dev); */

        watchdog_feed(WATCHDOG_TASK_GPS);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));  /* 10Hz */
    }
}

/* ══════════════════════════════════════════════════════
 *  Log task — 가변 (유휴 시간 활용)
 * ══════════════════════════════════════════════════════ */

static void log_task(void *param)
{
    (void)param;

    /* TODO: sd_logger_init() */

    for (;;)
    {
        /* TODO: 로그 데이터 기록 */

        watchdog_feed(WATCHDOG_TASK_LOG);

        vTaskDelay(pdMS_TO_TICKS(100));  /* ~10Hz, 정확한 주기 불필요 */
    }
}

/* ══════════════════════════════════════════════════════
 *  Application entry point
 * ══════════════════════════════════════════════════════ */

void app_main(void)
{
    /* 워치독 초기화 */
    watchdog_init();

    /* FreeRTOS 태스크 생성 */
    xTaskCreate(sensor_task,   "sensor",   STACK_SENSOR,   NULL, TASK_PRIO_SENSOR,   &sensor_task_handle);
    xTaskCreate(ahrs_task,     "ahrs",     STACK_AHRS,     NULL, TASK_PRIO_AHRS,     &ahrs_task_handle);
    xTaskCreate(control_task,  "control",  STACK_CONTROL,  NULL, TASK_PRIO_CONTROL,  &control_task_handle);
    xTaskCreate(watchdog_task_func, "wdog", STACK_WATCHDOG, NULL, TASK_PRIO_WATCHDOG, &watchdog_task_handle);
    xTaskCreate(mavlink_task,  "mavlink",  STACK_MAVLINK,  NULL, TASK_PRIO_MAVLINK,  &mavlink_task_handle);
    xTaskCreate(gps_task,      "gps",      STACK_GPS,      NULL, TASK_PRIO_GPS,      &gps_task_handle);
    xTaskCreate(log_task,      "log",      STACK_LOG,      NULL, TASK_PRIO_LOG,      &log_task_handle);

    /* 스케줄러 시작 (여기서 리턴하지 않음) */
    vTaskStartScheduler();

    /* 여기 도달하면 스케줄러 시작 실패 */
    error_handler_fatal(ERR_SCHEDULER_FAILED);
}
