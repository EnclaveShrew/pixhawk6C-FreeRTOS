/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

    /* Private includes ----------------------------------------------------------*/
    /* USER CODE BEGIN Includes */

    /* USER CODE END Includes */

    /* Exported types ------------------------------------------------------------*/
    /* USER CODE BEGIN ET */

    /* USER CODE END ET */

    /* Exported constants --------------------------------------------------------*/
    /* USER CODE BEGIN EC */

    /* USER CODE END EC */

    /* Exported macro ------------------------------------------------------------*/
    /* USER CODE BEGIN EM */

    /* USER CODE END EM */

    void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

    /* Exported functions prototypes ---------------------------------------------*/
    void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FMU_UART8_TX_GPS2_Pin GPIO_PIN_1
#define FMU_UART8_TX_GPS2_GPIO_Port GPIOE
#define FMU_UART8_RX_GPS2_Pin GPIO_PIN_0
#define FMU_UART8_RX_GPS2_GPIO_Port GPIOE
#define FMU_I2C1_SCL_Pin GPIO_PIN_8
#define FMU_I2C1_SCL_GPIO_Port GPIOB
#define FMU_CAN2_RX_Pin GPIO_PIN_5
#define FMU_CAN2_RX_GPIO_Port GPIOB
#define FMU_SDMMC2_D3_Pin GPIO_PIN_4
#define FMU_SDMMC2_D3_GPIO_Port GPIOB
#define FMU_SDMMC2_D2_Pin GPIO_PIN_3
#define FMU_SDMMC2_D2_GPIO_Port GPIOB
#define FMU_SDMMC2_CMD_Pin GPIO_PIN_7
#define FMU_SDMMC2_CMD_GPIO_Port GPIOD
#define FMU_UART5_TX_TEL2_Pin GPIO_PIN_12
#define FMU_UART5_TX_TEL2_GPIO_Port GPIOC
#define N_BRICK1_VALID_Pin GPIO_PIN_15
#define N_BRICK1_VALID_GPIO_Port GPIOA
#define FMU_SWCLK_Pin GPIO_PIN_14
#define FMU_SWCLK_GPIO_Port GPIOA
#define FMU_SWDIO_Pin GPIO_PIN_13
#define FMU_SWDIO_GPIO_Port GPIOA
#define FMU_HEATER_Pin GPIO_PIN_9
#define FMU_HEATER_GPIO_Port GPIOB
#define FMU_I2C1_SDA_Pin GPIO_PIN_7
#define FMU_I2C1_SDA_GPIO_Port GPIOB
#define FMU_USART1_TX_GPS1_Pin GPIO_PIN_6
#define FMU_USART1_TX_GPS1_GPIO_Port GPIOB
#define FMU_SDMMC2_CK_Pin GPIO_PIN_6
#define FMU_SDMMC2_CK_GPIO_Port GPIOD
#define FMU_CAN1_RX_Pin GPIO_PIN_0
#define FMU_CAN1_RX_GPIO_Port GPIOD
#define N_VDD_5V_HIPOWER_OC_Pin GPIO_PIN_11
#define N_VDD_5V_HIPOWER_OC_GPIO_Port GPIOC
#define N_VDD_5V_HIPOWER_EN_Pin GPIO_PIN_10
#define N_VDD_5V_HIPOWER_EN_GPIO_Port GPIOC
#define FMU_USB_DP_Pin GPIO_PIN_12
#define FMU_USB_DP_GPIO_Port GPIOA
#define FMU_USART2_TX_TEL3_Pin GPIO_PIN_5
#define FMU_USART2_TX_TEL3_GPIO_Port GPIOD
#define FMU_CAN1_TX_Pin GPIO_PIN_1
#define FMU_CAN1_TX_GPIO_Port GPIOD
#define FMU_USB_DM_Pin GPIO_PIN_11
#define FMU_USB_DM_GPIO_Port GPIOA
#define FMU_SPI1_CS3_ICM42688_Pin GPIO_PIN_13
#define FMU_SPI1_CS3_ICM42688_GPIO_Port GPIOC
#define FMU_SPI2_CS_FRAM_Pin GPIO_PIN_4
#define FMU_SPI2_CS_FRAM_GPIO_Port GPIOD
#define FMU_SPI2_SCK_FRAM_Pin GPIO_PIN_3
#define FMU_SPI2_SCK_FRAM_GPIO_Port GPIOD
#define FMU_UART5_RX_TEL2_Pin GPIO_PIN_2
#define FMU_UART5_RX_TEL2_GPIO_Port GPIOD
#define FMU_UART1_RX_GPS1_Pin GPIO_PIN_10
#define FMU_UART1_RX_GPS1_GPIO_Port GPIOA
#define FMU_SPI1_CS2_BMI055_GYRO_Pin GPIO_PIN_14
#define FMU_SPI1_CS2_BMI055_GYRO_GPIO_Port GPIOC
#define FMU_VBUS_SENSE_Pin GPIO_PIN_9
#define FMU_VBUS_SENSE_GPIO_Port GPIOA
#define FMU_SPI1_CS1_BMI055_ACC_Pin GPIO_PIN_15
#define FMU_SPI1_CS1_BMI055_ACC_GPIO_Port GPIOC
#define FMU_UART5_CTS_TEL2_Pin GPIO_PIN_9
#define FMU_UART5_CTS_TEL2_GPIO_Port GPIOC
#define FMU_CH1_Pin GPIO_PIN_8
#define FMU_CH1_GPIO_Port GPIOA
#define FMU_UART5_RTS_TEL2_Pin GPIO_PIN_8
#define FMU_UART5_RTS_TEL2_GPIO_Port GPIOC
#define FMU_USART6_RX_FROM_IO_Pin GPIO_PIN_7
#define FMU_USART6_RX_FROM_IO_GPIO_Port GPIOC
#define FMU_USART6_TX_TO_IO_Pin GPIO_PIN_6
#define FMU_USART6_TX_TO_IO_GPIO_Port GPIOC
#define FMU_CH6_Pin GPIO_PIN_15
#define FMU_CH6_GPIO_Port GPIOD
#define HW_REV_SENSE_Pin GPIO_PIN_0
#define HW_REV_SENSE_GPIO_Port GPIOC
#define HW_VER_SENSE_Pin GPIO_PIN_1
#define HW_VER_SENSE_GPIO_Port GPIOC
#define FMU_SPI2_MISO_FRAM_Pin GPIO_PIN_2
#define FMU_SPI2_MISO_FRAM_GPIO_Port GPIOC
#define FMU_SPI2_MOSI_FRAM_Pin GPIO_PIN_3
#define FMU_SPI2_MOSI_FRAM_GPIO_Port GPIOC
#define VDD_3V3_SENSORS_EN_Pin GPIO_PIN_2
#define VDD_3V3_SENSORS_EN_GPIO_Port GPIOB
#define FMU_CH5_Pin GPIO_PIN_14
#define FMU_CH5_GPIO_Port GPIOD
#define FMU_I2C4_SDA_Pin GPIO_PIN_13
#define FMU_I2C4_SDA_GPIO_Port GPIOD
#define FMU_CH8_Pin GPIO_PIN_1
#define FMU_CH8_GPIO_Port GPIOA
#define FMU_CH7_Pin GPIO_PIN_0
#define FMU_CH7_GPIO_Port GPIOA
#define FMU_SCALED_V5_Pin GPIO_PIN_4
#define FMU_SCALED_V5_GPIO_Port GPIOA
#define FMU_BAT1_I_Pin GPIO_PIN_4
#define FMU_BAT1_I_GPIO_Port GPIOC
#define FMU_I2C4_SCL_Pin GPIO_PIN_12
#define FMU_I2C4_SCL_GPIO_Port GPIOD
#define FMU_LED_BLUE_Pin GPIO_PIN_11
#define FMU_LED_BLUE_GPIO_Port GPIOD
#define FMU_LED_RED_Pin GPIO_PIN_10
#define FMU_LED_RED_GPIO_Port GPIOD
#define FMU_BAT2_I_Pin GPIO_PIN_2
#define FMU_BAT2_I_GPIO_Port GPIOA
#define FMU_SPI1_MISO_SENSOR_Pin GPIO_PIN_6
#define FMU_SPI1_MISO_SENSOR_GPIO_Port GPIOA
#define FMU_SPI1_SCK_SENSOR_Pin GPIO_PIN_5
#define FMU_SPI1_SCK_SENSOR_GPIO_Port GPIOA
#define FMU_BAT1_V_Pin GPIO_PIN_5
#define FMU_BAT1_V_GPIO_Port GPIOC
#define N_BRICK2_VALID_Pin GPIO_PIN_12
#define N_BRICK2_VALID_GPIO_Port GPIOB
#define FMU_CAN2_TX_Pin GPIO_PIN_13
#define FMU_CAN2_TX_GPIO_Port GPIOB
#define FMU_USART3_RX_DEBUG_Pin GPIO_PIN_9
#define FMU_USART3_RX_DEBUG_GPIO_Port GPIOD
#define FMU_USART3_TX_DEBUG_Pin GPIO_PIN_8
#define FMU_USART3_TX_DEBUG_GPIO_Port GPIOD
#define FMU_USART2_RX_TEL3_Pin GPIO_PIN_3
#define FMU_USART2_RX_TEL3_GPIO_Port GPIOA
#define FMU_SPI1_MOSI_SENSOR_Pin GPIO_PIN_7
#define FMU_SPI1_MOSI_SENSOR_GPIO_Port GPIOA
#define FMU_BAT2_V_Pin GPIO_PIN_1
#define FMU_BAT2_V_GPIO_Port GPIOB
#define FMU_BUZZER_Pin GPIO_PIN_0
#define FMU_BUZZER_GPIO_Port GPIOB
#define FMU_I2C2_SCL_Pin GPIO_PIN_10
#define FMU_I2C2_SCL_GPIO_Port GPIOB
#define FMU_I2C2_SDA_Pin GPIO_PIN_11
#define FMU_I2C2_SDA_GPIO_Port GPIOB
#define FMU_SDMMC2_D0_Pin GPIO_PIN_14
#define FMU_SDMMC2_D0_GPIO_Port GPIOB
#define FMU_SDMMC2_D1_Pin GPIO_PIN_15
#define FMU_SDMMC2_D1_GPIO_Port GPIOB

    /* USER CODE BEGIN Private defines */

    /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
