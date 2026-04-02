/**
 * @file main.c
 * @brief FreeRTOS task initialization and entry point
 *
 * Requirement: configTICK_RATE_HZ = 1000 (1ms tick)
 *   pdMS_TO_TICKS(1) must equal 1 for 1kHz tasks to work correctly.
 *   If tick rate < 1000, pdMS_TO_TICKS(1) = 0 -> 100% CPU usage.
 *
 * Task execution model:
 *   sensor/ahrs/control all run at 1kHz but with different priorities.
 *   sensor(7) completes -> yield -> ahrs(6) runs -> yield -> control(5) runs.
 *   Total execution time of all 3 tasks must stay under 1ms.
 *   If exceeded, lower-priority tasks starve -> watchdog detects it.
 *
 * Task priorities:
 *   [7] sensor_task     — 1kHz, IMU read (highest)
 *   [6] ahrs_task       — 1kHz, attitude estimation
 *   [5] control_task    — 1kHz(rate) / 250Hz(attitude), control loop
 *   [3] mavlink_task    — 50Hz, telemetry TX/RX
 *   [2] gps_task        — 10Hz, GPS parsing
 *   [1] log_task        — variable, SD card logging (lowest)
 *   [4] watchdog_task   — 10Hz, task health monitoring
 */

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

/* 1kHz tasks will not work if configTICK_RATE_HZ is not 1000 */
#if configTICK_RATE_HZ < 1000
    #error "configTICK_RATE_HZ must be >= 1000 for 1kHz task scheduling"
#endif

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
#include "lqr_controller.h"
#include "indi_controller.h"
#include "motor_mixer.h"

/* Communication */
#include "mavlink_handler.h"

/* System */
#include "watchdog.h"
#include "error_handler.h"

/* Logging */
#include "sd_logger.h"

/* ══════════════════════════════════════════════════════
 *  Task priorities (higher = higher priority)
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
#define STACK_AHRS      2048    /* EKF matrix ops: predict ~800B + update ~850B + context */
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
 *  Shared data (protected by critical section)
 *
 *  vec3f_t (12 bytes) is not atomic, so
 *  reads/writes are protected with taskENTER_CRITICAL.
 *  Critical sections are a few us, no impact on real-time performance.
 * ══════════════════════════════════════════════════════ */

static vec3f_t g_accel;
static vec3f_t g_gyro;
static float g_roll, g_pitch, g_yaw;

static inline void shared_write_imu(const vec3f_t *accel, const vec3f_t *gyro)
{
    taskENTER_CRITICAL();
    g_accel = *accel;
    g_gyro = *gyro;
    taskEXIT_CRITICAL();
}

static inline void shared_read_imu(vec3f_t *accel, vec3f_t *gyro)
{
    taskENTER_CRITICAL();
    *accel = g_accel;
    *gyro = g_gyro;
    taskEXIT_CRITICAL();
}

static inline void shared_write_euler(float roll, float pitch, float yaw)
{
    taskENTER_CRITICAL();
    g_roll = roll;
    g_pitch = pitch;
    g_yaw = yaw;
    taskEXIT_CRITICAL();
}

static inline void shared_read_euler(float *roll, float *pitch, float *yaw)
{
    taskENTER_CRITICAL();
    *roll = g_roll;
    *pitch = g_pitch;
    *yaw = g_yaw;
    taskEXIT_CRITICAL();
}

/* Active controller (swappable at runtime) */
static controller_t *g_active_ctrl = &cascade_pid_controller;

/*
 * Controller switch request (mavlink_task -> control_task)
 * MAVLink callback only sets a flag; actual switch is done in control_task.
 * control_task performs the switch safely at the start of its loop.
 * -> reset/init and update run sequentially in the same task, no race condition.
 */
/*
 * int8_t store is atomic on ARM Cortex-M (single byte store instruction).
 * For portability, stdatomic.h or sig_atomic_t would be preferable,
 * but avoided here due to IAR EWARM C11 mode dependency.
 */
static volatile int8_t g_ctrl_switch_request = -1;  /* -1 = no request, 0~2 = target type */

/* ══════════════════════════════════════════════════════
 *  Sensor task — 1kHz
 * ══════════════════════════════════════════════════════ */

static void sensor_task(void *param)
{
    (void)param;
    TickType_t last_wake = xTaskGetTickCount();

    /* TODO: Initialize spi_dev_t, i2c_dev_t (connect CubeMX-generated handles) */

    for (;;)
    {
        /* Read IMU (Primary: ICM-42688-P) */
        vec3f_t accel, gyro;
        /* if (icm42688p_read_accel_gyro(&imu_dev, &accel, &gyro) == SENSOR_OK) */
        /* { */
        /*     shared_write_imu(&accel, &gyro); */
        /*     watchdog_feed(WATCHDOG_TASK_SENSOR);  <- feed only on successful read */
        /* } */

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

    uint32_t prev_cycles = 0;
    uint8_t first_run = 1;

    for (;;)
    {
        vec3f_t accel, gyro;
        shared_read_imu(&accel, &gyro);

        /* Measure actual elapsed time via DWT cycle counter */
        uint32_t now_cycles = DWT->CYCCNT;
        if (first_run)
        {
            /* First frame: stamp time only, skip EKF to avoid dt spike */
            prev_cycles = now_cycles;
            first_run = 0;
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
            continue;
        }
        float dt = (float)(now_cycles - prev_cycles) / (float)SystemCoreClock;
        prev_cycles = now_cycles;

        ekf_predict(&ekf, &gyro, dt);
        ekf_update_accel(&ekf, &accel);
        /* ekf_update_mag(&ekf, &mag); — when magnetometer data available */

        float roll, pitch, yaw;
        ekf_get_euler(&ekf, &roll, &pitch, &yaw);
        shared_write_euler(roll, pitch, yaw);

        /* EKF computation completion means AHRS is operating normally */
        watchdog_feed(WATCHDOG_TASK_AHRS);

        /*
         * Stack high water mark check (development only).
         * Remove or reduce frequency in production.
         * If watermark drops below ~100 words, increase STACK_AHRS.
         */
        /* UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL); */
        /* if (hwm < 100) error_handler_report(ERR_STACK_OVERFLOW); */

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

    g_active_ctrl->init(g_active_ctrl->ctx, NULL);
    mixer_init(NULL);

    uint32_t ctrl_prev_cycles = 0;
    uint8_t ctrl_first_run = 1;

    for (;;)
    {
        /* First frame: stamp time only, skip control to avoid dt spike */
        if (ctrl_first_run)
        {
            ctrl_prev_cycles = DWT->CYCCNT;
            ctrl_first_run = 0;
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
            continue;
        }

        /* Process controller switch request (safe in control_task context) */
        int8_t req = g_ctrl_switch_request;
        if (req >= 0)
        {
            g_active_ctrl->reset(g_active_ctrl->ctx);
            switch (req)
            {
            case CTRL_TYPE_PID:
                g_active_ctrl = &cascade_pid_controller;
                break;
            case CTRL_TYPE_LQR:
                g_active_ctrl = &lqr_controller;
                break;
            case CTRL_TYPE_INDI:
                g_active_ctrl = &indi_controller;
                break;
            }
            g_active_ctrl->init(g_active_ctrl->ctx, NULL);
            g_ctrl_switch_request = -1;
        }

        controller_input_t input;
        controller_output_t output;

        /* Target attitude (set from RC input or autonomous flight) */
        input.target_attitude.w = 1.0f;
        input.target_attitude.x = 0.0f;
        input.target_attitude.y = 0.0f;
        input.target_attitude.z = 0.0f;

        /* Current attitude (AHRS output) */
        /* TODO: convert via euler_to_quat or get quaternion directly from EKF */
        input.current_attitude.w = 1.0f;
        input.current_attitude.x = 0.0f;
        input.current_attitude.y = 0.0f;
        input.current_attitude.z = 0.0f;

        vec3f_t cur_accel, cur_gyro;
        shared_read_imu(&cur_accel, &cur_gyro);
        input.gyro = cur_gyro;
        input.thrust = 0.0f;   /* Set from RC throttle */

        /* Measure actual dt via DWT cycle counter */
        uint32_t ctrl_now = DWT->CYCCNT;
        input.dt = (float)(ctrl_now - ctrl_prev_cycles) / (float)SystemCoreClock;
        ctrl_prev_cycles = ctrl_now;

        g_active_ctrl->update(g_active_ctrl->ctx, &input, &output);

        mixer_output_t motor_out;
        mixer_mix(&output, &motor_out);

        /* TODO: PWM output (via IO Processor or direct timer) */
        /* Move watchdog_feed after successful PWM output */

        /* Control computation complete = normal operation (until PWM output is implemented) */
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
    /* Handle received message: only set switch request, actual switch in control_task */
    if (msg->msg_id == MAVLINK_MSG_CTRL_SWITCH && msg->payload_len >= 1)
    {
        uint8_t target = msg->payload[0];
        if (target <= CTRL_TYPE_INDI)
        {
            g_ctrl_switch_request = (int8_t)target;
        }
    }
}

static void mavlink_task(void *param)
{
    (void)param;
    TickType_t last_wake = xTaskGetTickCount();

    /* TODO: Connect UART handle */
    /* mavlink_init(&g_mav, &telem_uart, mavlink_rx_callback); */

    for (;;)
    {
        /* Process received data */
        mavlink_poll(&g_mav);

        /* TX: Heartbeat 1Hz (every 50 loops) */
        if (g_mav_counter % 50 == 0)
        {
            mavlink_send_heartbeat(&g_mav,
                                   MAV_MODE_FLAG_STABILIZE | MAV_MODE_FLAG_CUSTOM,
                                   MAV_STATE_ACTIVE);
            mavlink_send_sys_status(&g_mav, 11100, 75, 0);
        }

        /* TX: Attitude 10Hz (every 5 loops) */
        if (g_mav_counter % 5 == 0)
        {
            float r, p, y;
            shared_read_euler(&r, &p, &y);
            mavlink_send_attitude(&g_mav, r, p, y, 0, 0, 0);
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

    /* TODO: Initialize ubx_dev_t */

    for (;;)
    {
        /* Feed only on successful GPS poll */
        /* if (ubx_poll(&gps_dev) > 0)  */
        /* {                             */
        /*     watchdog_feed(WATCHDOG_TASK_GPS); */
        /* }                             */

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));  /* 10Hz */
    }
}

/* ══════════════════════════════════════════════════════
 *  Log task — variable rate (uses idle time)
 * ══════════════════════════════════════════════════════ */

static void log_task(void *param)
{
    (void)param;

    /* TODO: sd_logger_init() */

    for (;;)
    {
        /* Feed only on successful log write */
        /* if (sd_logger_flush() == 0) */
        /* {                           */
        /*     watchdog_feed(WATCHDOG_TASK_LOG); */
        /* }                           */

        vTaskDelay(pdMS_TO_TICKS(100));  /* ~10Hz, exact period not required */
    }
}

/* ══════════════════════════════════════════════════════
 *  Application entry point
 * ══════════════════════════════════════════════════════ */

void app_main(void)
{
    /* Enable DWT cycle counter (used by ahrs_task and control_task for precise dt) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Initialize watchdog */
    watchdog_init();

    /* Create FreeRTOS tasks */
    xTaskCreate(sensor_task,   "sensor",   STACK_SENSOR,   NULL, TASK_PRIO_SENSOR,   &sensor_task_handle);
    xTaskCreate(ahrs_task,     "ahrs",     STACK_AHRS,     NULL, TASK_PRIO_AHRS,     &ahrs_task_handle);
    xTaskCreate(control_task,  "control",  STACK_CONTROL,  NULL, TASK_PRIO_CONTROL,  &control_task_handle);
    xTaskCreate(watchdog_task_func, "wdog", STACK_WATCHDOG, NULL, TASK_PRIO_WATCHDOG, &watchdog_task_handle);
    xTaskCreate(mavlink_task,  "mavlink",  STACK_MAVLINK,  NULL, TASK_PRIO_MAVLINK,  &mavlink_task_handle);
    xTaskCreate(gps_task,      "gps",      STACK_GPS,      NULL, TASK_PRIO_GPS,      &gps_task_handle);
    xTaskCreate(log_task,      "log",      STACK_LOG,      NULL, TASK_PRIO_LOG,      &log_task_handle);

    /* Start scheduler (should never return) */
    vTaskStartScheduler();

    /* Reaching here means scheduler failed to start */
    error_handler_fatal(ERR_SCHEDULER_FAILED);
}
