# Pixhawk 6C (FMUv6C) — STM32H743 핀맵

> **소스:** `FMUv6C_stm32_pinout_v1.pdf`
> **MCU:** STM32H743IIK6 (UFBGA176 → 실제 100pin 사용)
> **패키지:** UFBGA176 (핀아웃 문서 기준 ~100 I/O 사용)

---

## 1. CubeMX 프로젝트 설정 정보

### 1.1 MCU 기본 정보

| 항목 | 설정값 |
|---|---|
| MCU | STM32H743IIK6 |
| Core | Cortex-M7, 480 MHz |
| Flash | 2 MB |
| SRAM | 1 MB (DTCM 128KB, AXI SRAM 512KB, SRAM1~4) |

### 1.2 클럭 설정 상세 가이드

#### STM32H7 클럭 구조 개요 (vs STM32F4와의 차이)

STM32H743은 **3개의 전원/클럭 도메인**으로 분리되어 있다:

```
┌─────────────────────────────────────────────────────────┐
│                    D1 Domain (CPU)                       │
│  ┌─────────┐                                            │
│  │Cortex-M7│ ← sys_ck (480 MHz)                        │
│  └─────────┘                                            │
│  AXI bus ← HCLK3 (240 MHz)                             │
│  AHB3    ← HCLK3 (240 MHz)   [MDMA, Flash, FMC, QSPI] │
│  APB3    ← PCLK3 (120 MHz)   [LTDC, DSI]               │
├─────────────────────────────────────────────────────────┤
│                    D2 Domain (Peripherals)               │
│  AHB1    ← HCLK1 (240 MHz)   [DMA1, DMA2, ETH, USB]   │
│  AHB2    ← HCLK2 (240 MHz)   [DCMI, CRYP, RNG]        │
│  APB1    ← PCLK1 (120 MHz)   [TIM2~7,12~14, SPI2/3,   │
│                                USART2/3, UART4~8,       │
│                                I2C1~3, CAN1/2]          │
│  APB2    ← PCLK2 (120 MHz)   [TIM1,8,15~17, SPI1/4/5, │
│                                USART1/6, SAI1/2]        │
├─────────────────────────────────────────────────────────┤
│                    D3 Domain (Always-on)                 │
│  AHB4    ← HCLK4 (240 MHz)   [GPIO, RCC, EXTI, BDMA]  │
│  APB4    ← PCLK4 (120 MHz)   [I2C4, SPI6, LPTIM2~5,   │
│                                LPUART1, SAI4]           │
└─────────────────────────────────────────────────────────┘
```

> **핵심 차이:** STM32F4는 도메인 구분 없이 APB1/APB2만 나뉘었지만, H7은 D1/D2/D3 도메인별로 AHB와 APB가 각각 존재한다. 특히 **I2C4, SPI6은 D3 도메인(APB4)**에 있어서 다른 I2C/SPI와 클럭 소스가 다르다.

#### CubeMX Clock Configuration 탭 설정값

**Step 1: 클럭 소스 (RCC 설정)**

CubeMX 좌측 `System Core` → `RCC`에서 설정:

| 항목 | 설정값 | 위치 |
|---|---|---|
| HSE | Crystal/Ceramic Resonator | RCC → HSE |
| LSE | Crystal/Ceramic Resonator (실장 시) | RCC → LSE |
| HSI48 | Enabled (USB 48MHz 후보) | RCC → HSI48 |

> Pixhawk 6C 보드에 **16 MHz HSE 크리스탈**이 실장되어 있다 (Pixhawk 표준).

**Step 2: PLL 설정**

Clock Configuration 탭에서 아래 값을 입력/선택:

```
HSE (16 MHz)
  │
  ├──→ PLL1 (System PLL) ─── 메인 클럭
  │     DIVM1 = 1    → PLL1 입력 = 16 MHz
  │     MULN1 = 60   → VCO = 16 × 60 = 960 MHz
  │     DIVP1 = 2    → PLL1P = 480 MHz → SYSCLK
  │     DIVQ1 = 5    → PLL1Q = 192 MHz → SPI1/2/3 커널 클럭
  │     DIVR1 = 2    → PLL1R = 480 MHz → (Trace 등)
  │
  ├──→ PLL2 ─── 선택사항 (특정 페리페럴 커널 클럭)
  │     필요 시 SPI, ADC 등의 커널 클럭 소스로 활용
  │
  ├──→ PLL3 ─── USB 48MHz
  │     DIVM3 = 1    → PLL3 입력 = 16 MHz
  │     MULN3 = 24   → VCO = 16 × 24 = 384 MHz
  │     DIVP3 = 8    → PLL3P = 48 MHz → (SAI 등)
  │     DIVQ3 = 8    → PLL3Q = 48 MHz → USB
  │     DIVR3 = 8    → PLL3R = 48 MHz
  │
  └──→ HSI48 (48 MHz) → USB 대체 소스 (CRS로 SOF 동기화 가능)
```

> **TIP:** CubeMX에서 SYSCLK 입력란에 `480`을 직접 입력하면 PLL 값을 자동 계산해준다. 단, 자동 계산 결과가 위 값과 다를 수 있으니 확인 필요.

**Step 3: 버스 분주비 (Clock Configuration 탭 우측)**

| 클럭 | 분주비 | 최종 주파수 | CubeMX 필드 |
|---|---|---|---|
| SYSCLK | — | 480 MHz | System Clock Mux → PLLCLK |
| D1CPRE (CPU) | /1 | 480 MHz | |
| HPRE (AHB) | /2 | 240 MHz | AHB Prescaler |
| D1PPRE (APB3) | /2 | 120 MHz | APB3 Prescaler |
| D2PPRE1 (APB1) | /2 | 120 MHz | APB1 Prescaler |
| D2PPRE2 (APB2) | /2 | 120 MHz | APB2 Prescaler |
| D3PPRE (APB4) | /2 | 120 MHz | APB4 Prescaler |

> **타이머 클럭 주의:** APB 분주비가 1이 아닐 때, 타이머 클럭은 APBx × 2이다.
> - APB1 = 120 MHz → TIM2~7, TIM12~14 클럭 = **240 MHz**
> - APB2 = 120 MHz → TIM1, TIM8, TIM15~17 클럭 = **240 MHz**
>
> STM32F4와 동일한 규칙이지만, H7에서는 `rcc_timx_ker_ck` multiplier 설정을 확인할 것.

**Step 4: 페리페럴 커널 클럭 (Peripheral Clock Selection)**

STM32H7의 핵심 차이점: 많은 페리페럴이 **버스 클럭과 별도로 커널 클럭을 선택**할 수 있다.

CubeMX `System Core` → `RCC` → 하단 또는 Clock Configuration 탭에서 설정:

| 페리페럴 | 커널 클럭 소스 | 주파수 | 근거 |
|---|---|---|---|
| SPI1/2/3 (D2) | PLL1Q | 192 MHz (DIVQ1=5) | SPI 프리스케일러로 최종 SCK 결정 |
| SPI6 (D3) | PCLK4 | 120 MHz | D3 도메인 |
| I2C1/2/3 (D2) | PCLK1 | 120 MHz | I2C 타이밍 레지스터로 400kHz 생성 |
| I2C4 (D3) | PCLK4 | 120 MHz | D3 도메인, 별도 커널 클럭 |
| USART1/6 (D2) | PCLK2 | 120 MHz | |
| USART2/3, UART4~8 (D2) | PCLK1 | 120 MHz | |
| FDCAN1/2 | HSE | 16 MHz | PSC=2, Seg1=5, Seg2=2 → 1Mbps |
| USB | PLL3Q | 48 MHz | PLL3: DIVM3=1, MULN3=24, DIVQ3=8 |
| ADC | PLL2P 또는 per_ck | 필요에 따라 | |
| SDMMC | PLL1Q 또는 PLL2R | 240 MHz 이하 | |

#### 최종 클럭 트리 요약

```
HSE 16 MHz
  │
  ├─ PLL1 ──→ SYSCLK = 480 MHz
  │            ├─ CPU      = 480 MHz  (/1)
  │            ├─ AHB1~4   = 240 MHz  (/2)
  │            ├─ APB1     = 120 MHz  (/2) → Timer2~7,12~14 = 240 MHz
  │            ├─ APB2     = 120 MHz  (/2) → Timer1,8,15~17 = 240 MHz
  │            ├─ APB3     = 120 MHz  (/2)
  │            └─ APB4     = 120 MHz  (/2)
  │
  ├─ PLL3Q ──→ USB = 48 MHz
  │
  └─ HSI48 ──→ USB 대체 (CRS 동기화)

페리페럴별 최종 클럭:
  SPI1 SCK  = 192MHz / prescaler (센서별 조정, 커널 클럭 = PLL1Q)
  I2C4      = 120MHz 기반 → 타이밍 레지스터로 400kHz
  USART6    = 120MHz → 1.5Mbps baudrate (오차 < 0.1%)
  TIM1 PWM  = 240MHz → PSC/ARR로 400Hz PWM
  FDCAN     = HSE 16MHz → 1Mbps 비트타이밍 (PSC=2, Seg1=5, Seg2=2)
```

#### CubeMX 설정 순서 요약

1. `RCC` → HSE: Crystal, HSI48: Enabled
2. `Clock Configuration` 탭 → SYSCLK에 `480` 입력 → Resolve 시도
3. PLL1 값 확인/수정 (DIVM=1, MULN=60, DIVP=2)
4. AHB/APB 분주비 확인 (AHB=/2, APB1~4=/2)
5. PLL3Q = 48MHz 설정 (USB용)
6. 페리페럴 커널 클럭 소스 확인

> **VOS (Voltage Output Scaling) 주의:** 480MHz 동작을 위해서는 **VOS0 (Overdrive)** 모드가 필요하다. CubeMX `System Core` → `PWR` → Voltage Scaling에서 `Scale 0` 선택. VOS1 이하에서는 최대 400MHz.

### 1.3 전원 설정 (PWR)

| 항목 | 설정값 | 근거 |
|---|---|---|
| Voltage Scaling | **Scale 0 (VOS0)** | 480 MHz 동작 필수 |
| Supply Source | LDO (기본) | Pixhawk 보드 회로에 따라 |
| PVD | 선택사항 | 배터리 전압 감시 |

> VOS0 = 1.26V 코어 전압, 최대 480 MHz 허용. VOS1 = 최대 400 MHz, VOS2 = 300 MHz, VOS3 = 200 MHz.

### 1.4 디버그

| 항목 | 설정값 |
|---|---|
| Debug | Serial Wire (SWD) |
| SWDIO | PA13 |
| SWCLK | PA14 |
| Debug Console | USART3 (PD8 TX, PD9 RX) |

### 1.3 부트 모드

| 모드 | BOOT0 | 용도 |
|---|---|---|
| Flash 부트 | Low (0) | 일반 동작 |
| System Memory (DFU) | High (1) + Reset | USB DFU 펌웨어 업로드 |

---

## 2. 센서 버스 핀맵

### 2.1 SPI1 — IMU (ICM-42688-P, BMI055)

| 기능 | 핀 | AF | CubeMX Label |
|---|---|---|---|
| SCK | PA5 | AF5 (SPI1) | FMU_SPI1_SCK_SENSOR |
| MISO | PA6 | AF5 (SPI1) | FMU_SPI1_MISO_SENSOR |
| MOSI | PA7 | AF5 (SPI1) | FMU_SPI1_MOSI_SENSOR |

**Chip Select (GPIO Output, Active Low):**

| 센서 | CS 핀 | CubeMX Label | 초기 상태 |
|---|---|---|---|
| BMI055 Accel | PC15 | FMU_SPI1_CS1_BMI055_ACC | High |
| BMI055 Gyro | PC14 | FMU_SPI1_CS2_BMI055_GYRO | High |
| ICM-42688-P | PC13 | FMU_SPI1_CS3_ICM42688 | High |

**Data Ready (GPIO Input, EXTI):**

| 센서 | DRDY 핀 | CubeMX Label | 트리거 |
|---|---|---|---|
| BMI055 Accel | PE4 | FMU_SPI1_DRDY1_BMI055_ACC | Rising Edge |
| BMI055 Gyro | PE5 | FMU_SPI1_DRDY2_BMI055_GYRO | Rising Edge |
| ICM-42688-P | PE6 | FMU_SPI1_DRDY3_ICM42688 | Rising Edge |

**SPI1 설정:**

| 항목 | 값 | 근거 |
|---|---|---|
| Mode | Full-Duplex Master | |
| Prescaler | APB2/8 = 15 MHz (초기화), APB2/2 = 60 MHz (동작) | ICM-42688-P: 최대 24 MHz, BMI055: 최대 10 MHz |
| CPOL | High (1) | ICM-42688-P, BMI055 모두 SPI Mode 3 |
| CPHA | 2nd Edge (1) | SPI Mode 3 (CPOL=1, CPHA=1) |
| Data Size | 8-bit | |
| First Bit | MSB | |
| NSS | Software | CS를 GPIO로 수동 제어 |

> **주의:** ICM-42688-P(최대 24MHz)와 BMI055(최대 10MHz)가 SPI1 버스를 공유한다. 동작 클럭은 BMI055 기준 10MHz 이하로 설정하거나, 센서별로 프리스케일러를 동적 변경해야 한다.

### 2.2 SPI2 — FRAM

| 기능 | 핀 | AF | CubeMX Label |
|---|---|---|---|
| SCK | PD3 | AF5 (SPI2) | FMU_SPI2_SCK_FRAM |
| MISO | PC2 | AF5 (SPI2) | FMU_SPI2_MISO_FRAM |
| MOSI | PC3 | AF5 (SPI2) | FMU_SPI2_MOSI_FRAM |
| CS | PD4 | GPIO Output | FMU_SPI2_CS_FRAM |

**SPI2 설정:**

| 항목 | 값 |
|---|---|
| Mode | Full-Duplex Master |
| CPOL/CPHA | Mode 0 (CPOL=0, CPHA=0) 또는 Mode 3 (FRAM 데이터시트 확인) |
| NSS | Software |

### 2.3 I2C4 — IST8310, MS5611, 24LC64 EEPROM

| 기능 | 핀 | AF | CubeMX Label |
|---|---|---|---|
| SCL | PD12 | AF4 (I2C4) | FMU_I2C4_SCL |
| SDA | PD13 | AF4 (I2C4) | FMU_I2C4_SDA |

**I2C4 디바이스:**

| 디바이스 | 7-bit 주소 | 용도 |
|---|---|---|
| IST8310 | 0x0E | 내장 자력계 |
| MS5611 | 0x77 (CSB=Low) 또는 0x76 (CSB=High) | 바로미터 |
| 24LC64 | 0x50~0x57 (A0~A2 핀 의존) | EEPROM |

**I2C4 설정:**

| 항목 | 값 | 근거 |
|---|---|---|
| Speed Mode | Fast Mode (400 kHz) | IST8310, MS5611 모두 400kHz 지원 |
| Rise/Fall Time | 보드 특성에 따라 조정 | |
| Pull-up | 외부 풀업 (보드에 실장) | |

### 2.4 I2C1 — GPS1 외부 자력계/LED

| 기능 | 핀 | AF | CubeMX Label |
|---|---|---|---|
| SCL | PB8 | AF4 (I2C1) | FMU_I2C1_SCL_GPS1_MAG_LED |
| SDA | PB7 | AF4 (I2C1) | FMU_I2C1_SDA_GPS1_MAG_LED |

**I2C1 설정:** Fast Mode (400 kHz)

### 2.5 I2C2 — GPS2 외부 자력계/LED

| 기능 | 핀 | AF | CubeMX Label |
|---|---|---|---|
| SCL | PB10 | AF4 (I2C2) | FMU_I2C2_SCL_GPS2_MAG_LED |
| SDA | PB11 | AF4 (I2C2) | FMU_I2C2_SDA_GPS2_MAG_LED |

**I2C2 설정:** Fast Mode (400 kHz)

---

## 3. UART/USART 핀맵

| 페리페럴 | 용도 | TX 핀 | RX 핀 | RTS | CTS | AF | Flow Control | Baudrate (기본) |
|---|---|---|---|---|---|---|---|---|
| USART1 | GPS1 | PB6 | PA10 | — | — | AF7 | None | 38400 (UBX) |
| USART2 | TEL3 | PD5 | PA3 | — | — | AF7 | None | 57600 |
| USART3 | DEBUG | PD8 | PD9 | — | — | AF7 | None | 57600 |
| UART5 | TEL2 | PC12 | PD2 | PC8 | PC9 | AF8 | HW (RTS/CTS) | 57600 |
| USART6 | IO Processor | PC6 | PC7 | — | — | AF7 | None | 1500000 |
| UART7 | TEL1 | PE8 | PE7 | PE9 | PE10 | AF7 | HW (RTS/CTS) | 57600 |
| UART8 | GPS2 | PE1 | PE0 | — | — | AF8 | None | 38400 (UBX) |

**CubeMX UART 공통 설정:**

| 항목 | 값 |
|---|---|
| Word Length | 8 Bits |
| Stop Bits | 1 |
| Parity | None |
| Mode | TX/RX |
| Oversampling | 16 |

> **USART6 (IO Processor 통신):** FMU↔STM32F103 간 고속 통신. PX4 기준 1.5 Mbps. 바이너리 프로토콜 사용.

---

## 4. PWM 출력 핀맵

### 4.1 FMU PWM (8채널, STM32H743 직접 출력)

| 채널 | 핀 | 타이머 | AF | CubeMX Label |
|---|---|---|---|---|
| CH1 | PA8 | TIM1_CH1 | AF1 | FMU_CH1 |
| CH2 | PE11 | TIM1_CH2 | AF1 | FMU_CH2 |
| CH3 | PE13 | TIM1_CH3 | AF1 | FMU_CH3 |
| CH4 | PE14 | TIM1_CH4 | AF1 | FMU_CH4 |
| CH5 | PD14 | TIM4_CH3 | AF2 | FMU_CH5 |
| CH6 | PD15 | TIM4_CH4 | AF2 | FMU_CH6 |
| CH7 | PA0 | TIM5_CH1 | AF2 | FMU_CH7 |
| CH8 | PA1 | TIM5_CH2 | AF2 | FMU_CH8 |

**PWM 타이머 설정:**

| 타이머 | 채널 | Clock Source | Prescaler | Period | PWM 주파수 | 근거 |
|---|---|---|---|---|---|---|
| TIM1 | CH1~CH4 | APB2 (240 MHz, x2=480 MHz) | 479 | 1999 | 400 Hz (Oneshot 시 변경) | 멀티콥터 ESC 표준 |
| TIM4 | CH3~CH4 | APB1 (240 MHz, x2=480 MHz) | 479 | 1999 | 400 Hz | |
| TIM5 | CH1~CH2 | APB1 (240 MHz, x2=480 MHz) | 479 | 1999 | 400 Hz | |

> **참고:** ESC 프로토콜에 따라 주파수 변경 — 일반 PWM: 50~400Hz, Oneshot125: ~4kHz, DShot: 디지털 프로토콜.

### 4.2 IO Processor PWM (8채널, STM32F103 통해 출력)

- IO Processor(STM32F103)가 별도로 8ch PWM 출력 관리
- FMU → USART6 → IO Processor → Timer → PWM OUT
- IO Processor 핀맵은 별도 분석 필요 (STM32F103 스케매틱)

---

## 5. CAN 버스 핀맵

| 페리페럴 | TX 핀 | RX 핀 | AF | 트랜시버 | CubeMX Label |
|---|---|---|---|---|---|
| FDCAN1 | PD1 | PD0 | AF9 | TJA1051 | FMU_CAN1_TX / FMU_CAN1_RX |
| FDCAN2 | PB13 | PB5 | AF9 | TJA1051 | FMU_CAN2_TX / FMU_CAN2_RX |

**CAN 설정:**

| 항목 | 값 |
|---|---|
| Mode | FDCAN (또는 Classic CAN) |
| Nominal Bitrate | 1 Mbps (DroneCAN 표준) |
| Data Bitrate | 4~5 Mbps (FD 모드 시) |

---

## 6. USB 핀맵

| 기능 | 핀 | CubeMX Label |
|---|---|---|
| USB_OTG_FS_DM | PA11 | FMU_USB_DM |
| USB_OTG_FS_DP | PA12 | FMU_USB_DP |
| USB_OTG_FS_VBUS | PA9 | FMU_VBUS_SENSE |

**USB 설정:**

| 항목 | 값 |
|---|---|
| Mode | Device Only (OTG FS) |
| Speed | Full Speed (12 Mbps) |
| Class | CDC (Virtual COM Port) — 디버그/설정 콘솔 |

---

## 7. SDMMC 핀맵 (MicroSD)

| 기능 | 핀 | AF | CubeMX Label |
|---|---|---|---|
| CLK | PD6 | AF11 (SDMMC2) | FMU_SDMMC2_CK |
| CMD | PD7 | AF11 (SDMMC2) | FMU_SDMMC2_CMD |
| D0 | PB14 | AF9 (SDMMC2) | FMU_SDMMC2_D0 |
| D1 | PB15 | AF9 (SDMMC2) | FMU_SDMMC2_D1 |
| D2 | PB3 | AF9 (SDMMC2) | FMU_SDMMC2_D2 |
| D3 | PB4 | AF9 (SDMMC2) | FMU_SDMMC2_D3 |

**SDMMC2 설정:**

| 항목 | 값 |
|---|---|
| Bus Width | 4-bit |
| Clock Divider | 동적 조정 (초기화 400kHz → 동작 24~48MHz) |

---

## 8. ADC 핀맵

| 핀 | ADC 채널 | 용도 | CubeMX Label |
|---|---|---|---|
| PA2 | ADC1_INP14 | 배터리2 전류 | FMU_BAT2_I |
| PA4 | ADC1_INP18 | 5V 스케일드 전압 | FMU_SCALED_V5 |
| PB1 | ADC1_INP5 | 배터리2 전압 | FMU_BAT2_V |
| PC4 | ADC1_INP4 | 배터리1 전류 | FMU_BAT1_I |
| PC5 | ADC1_INP8 | 배터리1 전압 | FMU_BAT1_V |
| PC0 | ADC3_INP10 | HW Revision | HW_REV_SENSE |
| PC1 | ADC3_INP11 | HW Version | HW_VER_SENSE |

**ADC 설정:**

| 항목 | 값 |
|---|---|
| Resolution | 16-bit |
| Sampling Time | 센서별 조정 |
| Conversion Mode | DMA (배터리 모니터링은 주기적 폴링 또는 DMA) |

---

## 9. GPIO 핀맵

### 9.1 전원 제어 (GPIO Output)

| 핀 | 용도 | CubeMX Label | 초기 상태 | Active |
|---|---|---|---|---|
| PB2 | 3.3V 센서 전원 인에이블 | VDD_3V3_SENSORS_EN | High | High |
| PC10 | 5V 고전력 인에이블 | N_VDD_5V_HIPOWER_EN | High | Low (Active Low) |
| PE2 | 5V 페리페럴 인에이블 | N_VDD_5V_PERIPH_EN | High | Low (Active Low) |

### 9.2 전원 상태 감지 (GPIO Input)

| 핀 | 용도 | CubeMX Label | Active |
|---|---|---|---|
| PA15 | Brick1 전원 유효 | N_BRICK1_VALID | Low (Active Low) |
| PB12 | Brick2 전원 유효 | N_BRICK2_VALID | Low (Active Low) |
| PC11 | 5V 고전력 과전류 | N_VDD_5V_HIPOWER_OC | Low (Active Low) |
| PE3 | 5V 페리페럴 과전류 | N_VDD_5V_PERIPH_OC | Low (Active Low) |
| PE15 | USB VBUS 유효 | N_USB_VBUS_VALID | Low (Active Low) |

### 9.3 LED (GPIO Output, Active Low)

| 핀 | 색상 | CubeMX Label | Active |
|---|---|---|---|
| PD10 | RED | N_FMU_LED_RED | Low (Active Low) |
| PD11 | BLUE | N_FMU_LED_BLUE | Low (Active Low) |

### 9.4 기타

| 핀 | 기능 | 타이머/AF | CubeMX Label |
|---|---|---|---|
| PB0 | Buzzer | TIM3_CH3 (AF2) | FMU_BUZZER |
| PB9 | IMU Heater | TIM17_CH1 (AF1) | FMU_HEATER |
| PE12 | HW Version/Rev Drive | GPIO Output | HW_VER_REV_DRIVE |

---

## 10. FMU ↔ IO Processor (STM32F103) 통신

```
STM32H743 (FMU)              STM32F103 (IO)
┌──────────────┐              ┌──────────────┐
│ USART6_TX ───┼── PC6 ──────┼── UART2_RX   │
│ USART6_RX ───┼── PC7 ──────┼── UART2_TX   │
└──────────────┘              └──────────────┘
```

| 항목 | 값 |
|---|---|
| FMU 측 | USART6 (PC6 TX, PC7 RX) |
| IO 측 | UART2 |
| Baudrate | 1,500,000 bps |
| 프로토콜 | 바이너리 (PX4 px4io 프로토콜 참고 가능) |

### IO Processor 기능 (블록다이어그램 기준)

- **PWM 출력:** 8ch (별도 커넥터)
- **RC 입력:** PPM/SBUS IN (인버터 경유)
- **RSSI:** SBUS OUT / RSSI IN (인버터 경유)
- **DSM:** DSM-IN (UART1_RX)
- **Safety:** Safety LED + Safety Switch (→ GPS1 커넥터로 연결)
- **Spektrum:** Spektrum Power Enable
- **상태:** IO Status LED, VDD Servo Sense
- **디버그:** SWD, GPIO&SWO, UART1_TX, Timer

---

## 11. 전체 핀 할당 요약 (CubeMX 설정용)

### PA 포트

| 핀 | Mode | AF/설정 | Label |
|---|---|---|---|
| PA0 | TIM5_CH1 | AF2 | FMU_CH7 |
| PA1 | TIM5_CH2 | AF2 | FMU_CH8 |
| PA2 | ADC1_INP14 | Analog | FMU_BAT2_I |
| PA3 | USART2_RX | AF7 | FMU_USART2_RX_TEL3 |
| PA4 | ADC1_INP18 | Analog | FMU_SCALED_V5 |
| PA5 | SPI1_SCK | AF5 | FMU_SPI1_SCK_SENSOR |
| PA6 | SPI1_MISO | AF5 | FMU_SPI1_MISO_SENSOR |
| PA7 | SPI1_MOSI | AF5 | FMU_SPI1_MOSI_SENSOR |
| PA8 | TIM1_CH1 | AF1 | FMU_CH1 |
| PA9 | GPIO Input | — | FMU_VBUS_SENSE |
| PA10 | USART1_RX | AF7 | FMU_UART1_RX_GPS1 |
| PA11 | USB_OTG_FS_DM | AF10 | FMU_USB_DM |
| PA12 | USB_OTG_FS_DP | AF10 | FMU_USB_DP |
| PA13 | SWDIO | AF0 | FMU_SWDIO |
| PA14 | SWCLK | AF0 | FMU_SWCLK |
| PA15 | GPIO Input | — | N_BRICK1_VALID |

### PB 포트

| 핀 | Mode | AF/설정 | Label |
|---|---|---|---|
| PB0 | TIM3_CH3 | AF2 | FMU_BUZZER |
| PB1 | ADC1_INP5 | Analog | FMU_BAT2_V |
| PB2 | GPIO Output | Push-Pull, High | VDD_3V3_SENSORS_EN |
| PB3 | SDMMC2_D2 | AF9 | FMU_SDMMC2_D2 |
| PB4 | SDMMC2_D3 | AF9 | FMU_SDMMC2_D3 |
| PB5 | FDCAN2_RX | AF9 | FMU_CAN2_RX |
| PB6 | USART1_TX | AF7 | FMU_USART1_TX_GPS1 |
| PB7 | I2C1_SDA | AF4 | FMU_I2C1_SDA_GPS1_MAG_LED |
| PB8 | I2C1_SCL | AF4 | FMU_I2C1_SCL_GPS1_MAG_LED |
| PB9 | TIM17_CH1 | AF1 | FMU_HEATER |
| PB10 | I2C2_SCL | AF4 | FMU_I2C2_SCL_GPS2_MAG_LED |
| PB11 | I2C2_SDA | AF4 | FMU_I2C2_SDA_GPS2_MAG_LED |
| PB12 | GPIO Input | — | N_BRICK2_VALID |
| PB13 | FDCAN2_TX | AF9 | FMU_CAN2_TX |
| PB14 | SDMMC2_D0 | AF9 | FMU_SDMMC2_D0 |
| PB15 | SDMMC2_D1 | AF9 | FMU_SDMMC2_D1 |

### PC 포트

| 핀 | Mode | AF/설정 | Label |
|---|---|---|---|
| PC0 | ADC3_INP10 | Analog | HW_REV_SENSE |
| PC1 | ADC3_INP11 | Analog | HW_VER_SENSE |
| PC2 | SPI2_MISO | AF5 | FMU_SPI2_MISO_FRAM |
| PC3 | SPI2_MOSI | AF5 | FMU_SPI2_MOSI_FRAM |
| PC4 | ADC1_INP4 | Analog | FMU_BAT1_I |
| PC5 | ADC1_INP8 | Analog | FMU_BAT1_V |
| PC6 | USART6_TX | AF7 | FMU_USART6_TX_TO_IO |
| PC7 | USART6_RX | AF7 | FMU_USART6_RX_FROM_IO |
| PC8 | UART5_RTS | AF8 | FMU_UART5_RTS_TEL2 |
| PC9 | UART5_CTS | AF8 | FMU_UART5_CTS_TEL2 |
| PC10 | GPIO Output | Push-Pull, High | N_VDD_5V_HIPOWER_EN |
| PC11 | GPIO Input | — | N_VDD_5V_HIPOWER_OC |
| PC12 | UART5_TX | AF8 | FMU_UART5_TX_TEL2 |
| PC13 | GPIO Output | Push-Pull, High | FMU_SPI1_CS3_ICM42688 |
| PC14 | GPIO Output | Push-Pull, High | FMU_SPI1_CS2_BMI055_GYRO |
| PC15 | GPIO Output | Push-Pull, High | FMU_SPI1_CS1_BMI055_ACC |

### PD 포트

| 핀 | Mode | AF/설정 | Label |
|---|---|---|---|
| PD0 | FDCAN1_RX | AF9 | FMU_CAN1_RX |
| PD1 | FDCAN1_TX | AF9 | FMU_CAN1_TX |
| PD2 | UART5_RX | AF8 | FMU_UART5_RX_TEL2 |
| PD3 | SPI2_SCK | AF5 | FMU_SPI2_SCK_FRAM |
| PD4 | GPIO Output | Push-Pull, High | FMU_SPI2_CS_FRAM |
| PD5 | USART2_TX | AF7 | FMU_USART2_TX_TEL3 |
| PD6 | SDMMC2_CK | AF11 | FMU_SDMMC2_CK |
| PD7 | SDMMC2_CMD | AF11 | FMU_SDMMC2_CMD |
| PD8 | USART3_TX | AF7 | FMU_USART3_TX_DEBUG |
| PD9 | USART3_RX | AF7 | FMU_USART3_RX_DEBUG |
| PD10 | GPIO Output | Push-Pull, High | N_FMU_LED_RED |
| PD11 | GPIO Output | Push-Pull, High | N_FMU_LED_BLUE |
| PD12 | I2C4_SCL | AF4 | FMU_I2C4_SCL |
| PD13 | I2C4_SDA | AF4 | FMU_I2C4_SDA |
| PD14 | TIM4_CH3 | AF2 | FMU_CH5 |
| PD15 | TIM4_CH4 | AF2 | FMU_CH6 |

### PE 포트

| 핀 | Mode | AF/설정 | Label |
|---|---|---|---|
| PE0 | UART8_RX | AF8 | FMU_UART8_RX_GPS2 |
| PE1 | UART8_TX | AF8 | FMU_UART8_TX_GPS2 |
| PE2 | GPIO Output | Push-Pull, High | N_VDD_5V_PERIPH_EN |
| PE3 | GPIO Input | — | N_VDD_5V_PERIPH_OC |
| PE4 | GPIO Input (EXTI4) | Rising Edge | FMU_SPI1_DRDY1_BMI055_ACC |
| PE5 | GPIO Input (EXTI5) | Rising Edge | FMU_SPI1_DRDY2_BMI055_GYRO |
| PE6 | GPIO Input (EXTI6) | Rising Edge | FMU_SPI1_DRDY3_ICM42688 |
| PE7 | UART7_RX | AF7 | FMU_UART7_RX_TEL1 |
| PE8 | UART7_TX | AF7 | FMU_UART7_TX_TEL1 |
| PE9 | UART7_RTS | AF7 | FMU_UART7_RTS_TEL1 |
| PE10 | UART7_CTS | AF7 | FMU_UART7_CTS_TEL1 |
| PE11 | TIM1_CH2 | AF1 | FMU_CH2 |
| PE12 | GPIO Output | Push-Pull | HW_VER_REV_DRIVE |
| PE13 | TIM1_CH3 | AF1 | FMU_CH3 |
| PE14 | TIM1_CH4 | AF1 | FMU_CH4 |
| PE15 | GPIO Input | — | N_USB_VBUS_VALID |

---

## 12. DMA 할당 가이드 (CubeMX 참고)

| 페리페럴 | DMA Stream | 우선순위 | 근거 |
|---|---|---|---|
| SPI1 RX | DMA1 | Very High | IMU 고속 읽기 (1kHz) |
| SPI1 TX | DMA1 | Very High | |
| I2C4 RX | DMA1 | High | 바로미터/자력계 |
| I2C4 TX | DMA1 | High | |
| USART1 RX | DMA1 | Medium | GPS 수신 |
| UART7 TX | DMA1 | Medium | MAVLink 텔레메트리 |
| USART6 TX/RX | DMA1 | High | IO Processor 통신 |
| ADC1 | DMA2 | Low | 배터리 모니터링 |
| SDMMC2 | MDMA/IDMA | Low | SD 로깅 |

> **주의:** STM32H743은 DMA1/DMA2가 AXI SRAM에만 접근 가능. DTCM은 DMA 접근 불가. DMA 버퍼는 반드시 D1/D2 SRAM 영역에 배치해야 한다.

---

## 13. 인터럽트 우선순위 가이드 (NVIC, CubeMX 참고)

| 인터럽트 | 우선순위 (0=최고) | 근거 |
|---|---|---|
| EXTI4/5/6 (IMU DRDY) | 1 | 센서 데이터 최우선 |
| SPI1 DMA | 2 | IMU 데이터 전송 |
| TIM1/4/5 (PWM) | 3 | 모터 출력 |
| USART6 (IO Proc) | 4 | IO Processor 통신 |
| I2C4 DMA | 5 | 바로미터/자력계 |
| UART7 (TEL1) | 6 | MAVLink |
| USART1 (GPS) | 7 | GPS |
| ADC1 | 10 | 배터리 모니터링 |
| SDMMC2 | 12 | SD 로깅 |
| USB | 14 | USB CDC |

> **주의:** FreeRTOS 사용 시 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` (기본 5) 이상 우선순위의 ISR에서는 FreeRTOS API 호출 금지. IMU DRDY(1~2)는 FreeRTOS 관리 밖이므로 ISR 내에서 직접 플래그만 세팅하고, 태스크 통지로 전달.
