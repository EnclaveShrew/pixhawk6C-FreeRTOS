# Pixhawk 6C Bare-Metal Flight Controller

Pixhawk 6C + FreeRTOS 기반 FC Firmware 개발 프로젝트.

## Hardware

[Pixhawk 6C (Holybro)](https://holybro.com/products/pixhawk-6c)

## Project Structure

```
pixhawk6-bare-fc/
├── Core/                       ← CubeMX 생성 코드 (main, HAL config, IRQ)
├── Drivers/
│   ├── CMSIS/                  ← ARM CMSIS 라이브러리
│   ├── STM32H7xx_HAL_Driver/   ← ST HAL 라이브러리
│   ├── common/                 ← HAL 래퍼, 공통 타입
│   │   ├── spi_wrapper.c/.h    ← SPI 래퍼 (CS 관리 + HAL 추상화)
│   │   ├── sensor_types.h      ← vec3f_t, quat_t, error codes
│   │   └── debug_uart.c/.h     ← 디버그 UART 출력
│   └── imu/                    ← IMU 드라이버
│       ├── icm42688p.c/.h      ← Primary IMU (InvenSense, SPI)
│       └── bmi055.c/.h         ← Redundant IMU (Bosch, SPI, dual CS)
├── docs/
│   ├── pinmap.md               ← 스케매틱 기반 MCU-센서 핀맵
│   └── FMUv6C_stm32_pinout_v1.pdf
├── EWARM/                      ← IAR Embedded Workbench 프로젝트
├── pixhawk6-bare-fc.ioc        ← CubeMX 프로젝트 파일
└── README.md
```

## Build

### Requirements

- **IDE:** IAR Embedded Workbench for ARM (EWARM)
- **MCU:** STM32H743IIK6
- **CubeMX:** STM32CubeMX (pin configuration)

### Steps

1. Clone this repository
2. Open `EWARM/pixhawk6-bare-fc.ewp` in IAR EWARM
3. Build (F7)
4. Flash via SWD debugger (ST-Link / J-Link)

## License

MIT License
