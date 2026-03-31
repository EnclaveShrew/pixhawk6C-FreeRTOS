# Pixhawk 6C Bare-Metal Flight Controller

PX4/ArduPilot 없이, **센서 드라이버부터 AHRS, 제어기, MAVLink 통신까지 바닥부터 구현**하는 비행제어 펌웨어 프로젝트.

Pixhawk 6C 하드웨어 위에서 FreeRTOS 기반으로 동작하며, 다양한 제어 알고리즘(PID, LQR, INDI)을 런타임에 교체하며 비교 검증할 수 있는 **모듈형 구조**를 목표로 한다.

## Hardware

| 구성 요소 | 모델 | 인터페이스 |
|-----------|------|-----------|
| **FMU MCU** | STM32H743IIK6 (Cortex-M7, 480MHz) | - |
| **IO Processor** | STM32F103 (Cortex-M3, 72MHz) | UART |
| **Primary IMU** | ICM-42688-P (InvenSense) | SPI |
| **Redundant IMU** | BMI055 (Bosch) | SPI (CS x2) |
| **Barometer** | MS5611 (TE Connectivity) | I2C |
| **Magnetometer** | IST8310 (iSentek) | I2C |
| **GPS** | u-blox M10 (Holybro) | UART (UBX) |

## Architecture

```
 ┌─────────────────────────────────────────────────────────┐
 │                    FreeRTOS Tasks                        │
 │                                                         │
 │  Sensor Task ──→ AHRS Task ──→ Controller Task ──→ Mixer│
 │    (1kHz)         (1kHz)     (250Hz/1kHz)        (PWM)  │
 │                                                         │
 │  MAVLink Task (50Hz)    GPS Task (10Hz)    Logger Task  │
 └────────┬────────────────────┬───────────────────────────┘
          │                    │
 ┌────────┴────────┐  ┌───────┴────────┐
 │  Sensor Drivers  │  │  Communication  │
 │  ICM-42688-P     │  │  MAVLink v2     │
 │  BMI055          │  │  UBX Parser     │
 │  MS5611          │  │  Debug UART     │
 │  IST8310         │  └────────────────┘
 └────────┬────────┘
          │
 ┌────────┴────────┐
 │  HAL Wrappers    │
 │  spi_wrapper     │
 │  i2c_wrapper     │
 │  uart_wrapper    │
 └────────┬────────┘
          │
 ┌────────┴────────┐
 │  STM32 HAL       │
 │  (ST 제공)       │
 └──────────────────┘
```

### Controller Module Architecture

제어기를 공통 인터페이스(`controller_interface.h`)로 추상화하여, 런타임에 교체 가능한 플러그인 구조로 설계.

```
controller_interface.h
  ├── cascade_pid.c    (Cascade PID: attitude loop → rate loop)
  ├── lqr_controller.c (Linear-Quadratic Regulator)
  └── indi_controller.c(Incremental Nonlinear Dynamic Inversion)
         │
         ▼
    motor_mixer.c ──→ PWM Output
```

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

## Development Phases

| Phase | Description | Status |
|-------|------------|--------|
| **Phase 1** | Schematic analysis, pin map, CubeMX setup | Done |
| **Phase 2** | Sensor drivers (IMU, Baro, Mag, GPS) | In Progress |
| **Phase 3** | AHRS (Complementary filter, EKF) | - |
| **Phase 4** | Controllers (PID, LQR, INDI) + motor mixer | - |
| **Phase 5** | MAVLink v2 + companion computer (RPi) | - |
| **Phase 6** | FreeRTOS integration, failsafe, logging | - |

### Phase 2 Progress

| Driver | Sensor | Interface | Status |
|--------|--------|-----------|--------|
| ICM-42688-P | Primary IMU (6-axis) | SPI | Done |
| BMI055 | Redundant IMU (6-axis) | SPI (dual CS) | Done |
| MS5611 | Barometer | I2C | - |
| IST8310 | Magnetometer | I2C | - |
| u-blox M10 | GPS | UART (UBX) | - |

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

## Design Decisions

### Dual IMU with Different Manufacturers
ICM-42688-P (InvenSense)와 BMI055 (Bosch)를 조합하여, 동일 결함에 의한 동시 고장(common-mode failure)을 방지한다. Primary 고장 시 Redundant로 자동 전환.

### HAL Wrapper Layer
센서 드라이버와 STM32 HAL 사이에 래퍼 레이어를 두어, 전송 방식(polling → DMA) 변경 시 센서 드라이버 코드 수정 없이 래퍼 내부만 교체 가능.

### Controller Plugin Architecture
`controller_interface.h`로 공통 인터페이스를 정의하고, PID/LQR/INDI 제어기를 함수 포인터 테이블로 런타임 교체. 동일 비행 조건에서 제어 알고리즘 비교 검증 가능.

## License

MIT License
