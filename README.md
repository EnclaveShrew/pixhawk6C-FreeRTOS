# Pixhawk6C FreeRTOS Base Firmware Project

Pixhawk 6C + FreeRTOS 기반 FC Firmware 개발 프로젝트.

## Hardware

[Pixhawk 6C (Holybro)](https://holybro.com/products/pixhawk-6c)

## Build

### Requirements

- **IDE:** IAR Embedded Workbench for ARM (EWARM)
- **MCU:** STM32H743IIK6
- **CubeMX:** STM32CubeMX (pin configuration)

### Steps

1. Clone this repository
2. Open `EWARM/pixhawk6-bare-fc.ewp` in IAR EWARM
3. Build (F7)
4. Flash via USB-C (DFU mode: hold BOOT0 button + reset)

## License

MIT License
