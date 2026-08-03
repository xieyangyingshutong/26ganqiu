# STM32F103 Ball-on-Beam PID Control System

STM32F103 球杆 PID 控制系统，配合 K230 视觉模块和 Emm_V5 闭环步进驱动器，实现钢球在滑轨上的自动平衡与位置控制。

## Overview

```text
K230 Camera (YOLO11) ──USART3──> STM32F103 PID ──USART1──> Emm_V5 Stepper Driver ──> Ball-on-Beam Mechanism
           ball pos/vel & setpoint              beam angle             absolute pulse position
```

- **K230** 运行 YOLO11 目标检测，输出钢球位置、速度及目标位置（12 字节固定帧，115200 baud）
- **STM32F103** 执行 PID 控制律，包含 8 状态安全状态机、抗饱和积分、静摩擦补偿、设定值/倾角变化率限幅
- **Emm_V5** 闭环步进驱动器通过 `0xFD` 绝对位置命令控制曲柄角度

## Hardware Wiring

### K230 ↔ STM32

| K230 | STM32F103C8T6 |
|------|---------------|
| GPIO36 / UART4_TXD | PB11 / USART3_RX |
| GPIO37 / UART4_RXD | PB10 / USART3_TX (optional) |
| GND | GND |

### STM32 ↔ Emm_V5

| STM32F103C8T6 | Emm_V5 |
|---------------|--------|
| PA9 / USART1_TX | RX |
| PA10 / USART1_RX | TX (optional, current control doesn't depend on ACK) |
| GND | GND |

## State Machine

| Value | State | Description |
|------:|-------|-------------|
| 0 | WAITING_CAMERA | Waiting for first valid K230 packet |
| 1 | ARMING | Valid frames received, entering control |
| 2 | ACTIVE | Normal PID control |
| 3 | PREDICTED | Ball data not fresh, using prediction (up to 220ms) |
| 4 | EDGE_RECOVERY | Ball near track edge (up to 600ms) |
| 5 | BALL_INVALID | Ball or target data invalid |
| 6 | CAMERA_TIMEOUT | No valid packet for 500ms |
| 7 | MOTOR_FAULT | Motor link not ready |

## PID Control Law

```text
error = setpoint - ball_position
angle = Kp * error + integral - Kd * ball_velocity
```

| Parameter | Default | Unit |
|-----------|---------|------|
| Kp | 0.05000 | deg/mm |
| Ki | 0.00375 | deg/(mm·s) |
| Kd | 0.00875 | deg/(mm/s) |
| Max beam angle | ±8.0 | deg |
| Beam slew limit | 30 | deg/s |

## Key Features

- **8-state safety state machine** with prediction & timeout handling
- **Anti-windup integral** with conditional integration and deadband (1mm)
- **Stiction compensation** — detects stuck ball and ramps in temporary boost
- **Setpoint slew limiting** (80mm/s) to avoid step-response shocks
- **Motor command safety triad**: software angle limit + absolute pulse hard limits + slew rate limit
- **K230 XOR checksum validation** with byte-level state machine

## Configuration

All parameters in `APP/ball_control_config.h`:

- `BALL_MOTOR_DIRECTION_SIGN` — ±1.0, flip if ball accelerates away from target
- `BALL_MOTOR_PULSES_PER_BEAM_DEGREE` — mechanism calibration value
- `BALL_MOTOR_LEVEL_PULSES` / `MIN_PULSES` / `MAX_PULSES` — absolute pulse limits
- PID gains, stiction parameters, timeout values, etc.

## Build

```powershell
& 'D:\keil5\UV4\UV4.exe' -r 'PRJ\STM32_UART_CMD.uvprojx' -t 'STM32_PUL_WHILE'
```

Output: `PRJ/Objects/Template.hex`

## Project Structure

| Directory | Contents |
|-----------|----------|
| `APP/` | Main program, PID controller, control config, interrupt handlers |
| `BSP/` | Board init, K230 camera UART, Emm_V5 motor driver, USART1 driver |
| `DRIVERS/` | FIFO buffer, SysTick timebase, delay functions |
| `CMSIS/` | Cortex-M3 core support, startup code |
| `LIB/` | STM32F10x standard peripheral library |
| `PRJ/` | Keil uVision project file |

## PID Tuning Sequence

1. Confirm zero point, direction, gear ratio, mechanical limits
2. Set Ki=0, reduce max beam angle to 0.5–1.0°, verify negative feedback
3. Increase Kp until response is fast but no persistent oscillation
4. Increase Kd to suppress overshoot and oscillation
5. Finely increase Ki to eliminate steady-state error
6. Test mode 2 (sweep) and mode 3 (touch) after mode 1 is stable

## License

MIT