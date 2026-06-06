# Ens-492-Project-Low_Level_Control_Arcitecture-
A hybrid-locomotion robot controlled by 6 STM32F407 nodes over a 500 kbps CAN Bus, each running independent closed-loop PID on a single actuator. An ESP32 bridges WiFi to CAN for tether-free wireless operation. Powered by a 12V SLA battery with multi-rail 12V/5V/3.3V distribution

# Low-Level Controller & Electrical Architecture
### ENS 491–492 Graduation Project — Sabancı University
**Supervisor:** Prof. Kemalettin Erbatur | **Author:** Muhammed Burak Gülümser

> Part of a larger project: *Reinforcement Learning for a Hybrid-Locomotion Mobile Robot*
> — Can Karakurt (Mechanical Design) · Muhammed Burak Gülümser (Electrical & Low-Level Control) · Deniz Yurdakoç (RL Software & High-Level Controller)

---

## Table of Contents

- [System Overview](#system-overview)
- [Communication Architecture](#communication-architecture)
- [CAN Bus Protocol](#can-bus-protocol)
- [STM32 Node Firmware](#stm32-node-firmware)
  - [Keskinler Wiper Motor — Shoulder (Nodes 3 & 4)](#keskinler-wiper-motor--shoulder-nodes-3--4)
  - [JGB37 DC Motor — Wheels (Nodes 1 & 2)](#jgb37-dc-motor--wheels-nodes-1--2)
  - [Pololu Gearmotor — Elbow (Nodes 5 & 6)](#pololu-gearmotor--elbow-nodes-5--6)
- [ESP32 Gateway Firmware](#esp32-gateway-firmware)
- [Python Host Interface](#python-host-interface)
- [Power Architecture](#power-architecture)
- [Hardware Specifications](#hardware-specifications)
- [PID Parameters](#pid-parameters)
- [Wiring & Pin Assignments](#wiring--pin-assignments)
- [Getting Started](#getting-started)
- [Repository Structure](#repository-structure)

---

## System Overview

```
Host Computer  ──WiFi──►  ESP32 Gateway  ──CAN 500kbps──►  6× STM32F407 Nodes
(Python CLI)              (HTTP + TWAI)    SN65HVD230       (Encoder + PID + PWM)
                               ▲                                     │
                               └─────────── Feedback ────────────────┘
                                        position · velocity
```

Six **STM32F407 Discovery** boards each own exactly one actuator. Every node independently runs encoder acquisition, PID computation, and PWM generation. The **ESP32** acts purely as a communication gateway — it bridges WiFi HTTP requests from the host computer to CAN Bus messages and collects telemetry back from the nodes. No motor control logic runs on the ESP32.

---

## Communication Architecture

### ESP32 ↔ Host Computer (WiFi)

The ESP32 runs a **WiFi Access Point** (`ESP32_Motor_Net` / `password123`) and an HTTP server on port 80. Three endpoints are exposed:

| Endpoint | Method | Purpose |
|---|---|---|
| `/set?m=<1-6>&type=<pos\|vel\|kp\|ki\|kd>&val=<float>` | GET | Send a command to a motor |
| `/status?m=<1-6>` | GET | Read current state of a motor (JSON) |
| `/ping` | GET | CAN bus diagnostic — error counters, state |

`/status` response format:
```json
{"id":1, "tp":0.00, "cp":-1.23, "tv":0.0000, "cv":0.0012, "kp":30.00, "ki":2.000, "kd":0.000}
```

### ESP32 ↔ STM32 (CAN Bus)

- **Speed:** 500 kbps
- **Transceiver:** SN65HVD230 on every node (7 transceivers total)
- **Termination:** 120 Ω resistors at the two physical end nodes only
- **Prescaler:** 6, BS1: 11TQ, BS2: 2TQ (verified 500 kbps at 84 MHz APB1)

---

## CAN Bus Protocol

### Command Frame (ESP32 → STM32)

```
ID:  0x200 + MOTOR_ID       (0x201 … 0x206)
DLC: 5 bytes

Byte 0   : Command type (uint8)
Bytes 1-4: Value (float, little-endian)
```

| `Byte 0` | Command | Value unit |
|---|---|---|
| `0x00` | Position setpoint | degrees (float) |
| `0x01` | Velocity setpoint | rad/s (float) — wheel nodes only |
| `0x02` | Update Kp | float |
| `0x03` | Update Ki | float |
| `0x04` | Update Kd | float |

### Feedback Frame (STM32 → ESP32)

```
ID:  0x300 + MOTOR_ID       (0x301 … 0x306)
DLC: 8 bytes

Bytes 0-3: Current position (degrees, float)
Bytes 4-7: Current velocity (rad/s, float)
```

Feedback is transmitted every **100 ms** (shoulder nodes) / **50 ms** (wheel and elbow nodes).

---

## STM32 Node Firmware

All three firmware variants share the same structure:

```
startup → init timers (encoder + PWM) → init CAN → enable RX interrupt
main loop:
  ├─ every N ms  → read encoder → compute PID → set PWM
  └─ every 50-100ms → transmit CAN feedback
ISR (CAN RX FIFO0):
  └─ parse command type → update target / PID gains
```

Runtime PID gain updates take effect immediately — integral is reset on every gain change to prevent wind-up spikes on transition.

---

### Keskinler Wiper Motor — Shoulder (Nodes 3 & 4)

**File:** `embedded/shoulder/main.c` | **Motor IDs:** 3, 4

**Encoder:** 600 PPR external rotary encoder, quadrature (×4) = **2400 counts/rev**
**Timer:** TIM1 in `TIM_ENCODERMODE_TI12` (both edges, both channels)
**PWM:** TIM2 CH1/CH2, period = 100 (0–100% duty), prescaler = 41 → ~20 kHz
**Driver:** IBT-2 (BTS7960) — two complementary PWM channels (forward/reverse)

**Control loop:** 1 ms (1 kHz), triggered by `HAL_GetTick()` polling.

**Safety watchdog:** If no CAN command is received for **300 ms**, motor is stopped and integral is zeroed.

```c
// Encoder accumulation — signed delta from 16-bit counter wrap
void updateEncoder(void) {
    uint16_t current_cnt = __HAL_TIM_GET_COUNTER(&htim1);
    int16_t  delta       = (int16_t)(current_cnt - last_timer_cnt);
    encoderCount        += delta;
    last_timer_cnt       = current_cnt;
}
```

**Acceleration limiter:** Maximum position reference step of ±2 ticks/cycle applied before the PID to smooth large step commands.

**Anti-windup:** Integral term clamped so that `Ki × integral` never exceeds ±20 PWM counts.

```c
// Direct PWM clamping anti-windup
if ((Ki * integral) >  max_I_pwm) integral =  max_I_pwm / Ki;
if ((Ki * integral) < -max_I_pwm) integral = -max_I_pwm / Ki;
```

**Deadband:** Error < 5 ticks → motor stopped, integral reset.

**PID gains (final):** Kp = 0.8 · Ki = 0.02 · Kd = 0.000001

---

### JGB37 DC Motor — Wheels (Nodes 1 & 2)

**File:** `embedded/wheel/main.c` | **Motor IDs:** 1, 2

**Encoder:** 600 PPR external rotary encoder — `TIM_ENCODERMODE_TI1` (single-edge, ×1 = 600 counts/rev)
**Timer:** TIM3 encoder mode
**PWM:** TIM4 CH2, period = 4199 (0–4199 duty), prescaler = 0 → ~10 kHz
**Driver:** L298N — direction via GPIOB `MOTOR_INA_Pin` / `MOTOR_INB_Pin`, speed via PWM ENA

**Control loop:** 5 ms (200 Hz), timestamp-polled.

**Dual mode — position and velocity:**

```c
if (mode == 0) {   // Position mode
    error = current_target_pos - (float)absolute_pos;
} else {           // Velocity mode (shadow-setpoint)
    error = profile_velocity - current_vel;  // current_vel in ticks/s
}
```

Velocity commands (`0x01`) arrive as **rad/s**, converted to deg/s then to ticks/s:
```c
float target_vel_deg_s = value * 57.29578f;
profile_velocity = target_vel_deg_s / degrees_per_tick;
```

**Smooth position profile:** On every new position command, the delta is spread over 40 cycles (200 ms) as a linear interpolation ramp.

**Anti-windup:** Integral clamped at ±400 counts.

**CAN feedback:** Every 50 ms — position in degrees, velocity in rad/s.

**PID gains (final):** Kp = 30 · Ki = 2 · Kd = 0

---

### Pololu Gearmotor — Elbow (Nodes 5 & 6)

**File:** `embedded/elbow/main.c` | **Motor IDs:** 5, 6

**Encoder:** Built-in encoder, 64 PPR × gear ratio 131 = **8384 counts/rev**
**Timer:** TIM3 in `TIM_ENCODERMODE_TI12` with IC filter = 10 (noise rejection)
**PWM:** TIM1 CH1/CH2, period = 839 (0–839 duty), prescaler = 0 → ~100 kHz
**Driver:** IBT-2 — complementary channels
**Timing source:** TIM7 at 1 kHz interrupt for precise `ms_tick` counter (not HAL_GetTick)

**Control loop:** 10 ms (100 Hz) via `ms_tick`.

**Derivative on measurement** (not error) to eliminate derivative kick on setpoint changes:
```c
float deriv  = -(meas - p->prev_meas) / dt;
p->prev_meas = meas;
```

**Backlash compensation:** Direction change detected; a fixed offset (`BACKLASH_COUNTS = 100`) is added to the measured position momentarily to pre-load the gearbox backlash.

**Deadband:** Error < 1.0° → motor off, integral reset.

**PWM floor/ceiling:** Min duty = 60, max duty = 820 (of 839). Output below floor is treated as zero.

**Anti-windup:** Integral clamped at ±400 (in degree·s units).

**PID gains (final):** Kp = 40 · Ki = 10 · Kd = 2

---

## ESP32 Gateway Firmware

**File:** `embedded/esp32/main.ino`

### Motor profiles (loaded at startup)

```cpp
// Motors 1-2: JGB37 wheels
motors[1,2].kp = 30.0  · ki = 2.0  · kd = 0.0

// Motors 3-4: Keskinler shoulders
motors[3,4].kp = 0.8   · ki = 0.02 · kd = 0.000001

// Motors 5-6: Pololu elbows
motors[5,6].kp = 40.0  · ki = 10.0 · kd = 2.0
```

Profiles are written to the STM32 nodes at startup so all nodes begin with correct gains without requiring the host interface.

### CAN transmission

All commands are packed as 5-byte frames — 1 byte command type + 4 bytes float:
```cpp
void sendFloat(uint8_t cmdType, float val, int motorId) {
    msg.identifier = 0x200 + motorId;
    msg.data[0]    = cmdType;
    memcpy(&msg.data[1], &val, 4);
    twai_transmit(&msg, pdMS_TO_TICKS(10));
}
```

### CAN reception

Incoming feedback frames (`0x301`–`0x306`) are parsed in the main loop (non-blocking, 0 ms timeout):
```cpp
void handleCAN() {
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        int m = msg.identifier - 0x300;   // 1–6
        memcpy(&motors[m].currentDeg, &msg.data[0], 4);
        memcpy(&motors[m].currentVel, &msg.data[4], 4);
    }
}
```

### Serial monitor interface

A full serial command interface mirrors the HTTP API, useful for direct USB debugging:
```
motor<n>        Select active motor (1-6)
pos <deg>       Send position command
vel <rad/s>     Send velocity command
kp/ki/kd <val>  Update PID gain live
ping            Print CAN bus error counters
```

---

## Python Host Interface

**File:** `host_interface/motor_controller.py`

Runs two threads:
- **Fetch thread** — polls `/status` every 150 ms and prints live telemetry to stdout, matching the ESP32 serial monitor format exactly.
- **Main thread** — reads commands from stdin.

```
[M1] TarPos: 0.00° | CurPos: -1.23° | TarVel: 0.0000 rad/s | CurVel: 0.0012 rad/s | Kp:30.00 Ki:2.000 Kd:0.000
```

### Usage

```bash
pip install requests
python motor_controller.py

# Connect to WiFi: ESP32_Motor_Net / password123
# Then type commands:
motor1          # select wheel left
vel 2.5         # send 2.5 rad/s
motor5          # select elbow left
pos 90          # move to 90°
kp 45           # update Kp live
ping            # check CAN bus health
exit
```

### Programmatic use

```python
import requests

ESP = "http://192.168.4.1"

# Send position command to motor 5 (elbow left)
requests.get(f"{ESP}/set", params={"m": 5, "type": "pos", "val": 90.0})

# Read current state
r = requests.get(f"{ESP}/status", params={"m": 5})
state = r.json()
print(state["cp"])   # current position in degrees
```

---

## Power Architecture

```
12V 1.3Ah SLA Battery (also structural rear counterweight)
        │
        ├──► 12V ──► IBT-2 drivers (Pololu elbows + Keskinler shoulders)
        │    12V ──► L298N drivers (JGB37 wheels)
        │
        └──► L298N 5V regulator
                  │
                  ├──► 5V ──► All 6 × STM32F407 boards
                  ├──► 5V ──► ESP32 module
                  ├──► 5V ──► 6 × SN65HVD230 CAN transceivers
                  └──► 5V ──► External rotary encoders
                              │
                              └──► 3.3V (from STM32 / ESP32 pins)
                                        └──► Logic-level peripherals
```

No external DC-DC step-down modules required. The L298N's built-in 5V regulator powers the entire logic tier, keeping the BOM minimal.

---

## Hardware Specifications

| Component | Model | Qty | Motor IDs | Notes |
|---|---|---|---|---|
| Microcontroller | STM32F407 Discovery | 6 | 1–6 | One per actuator |
| Gateway | ESP32 DevKit | 1 | — | WiFi AP + CAN bridge |
| CAN Transceiver | SN65HVD230 | 7 | — | 120 Ω termination at end nodes |
| Shoulder Motor | Keskinler Wiper Motor | 2 | 3, 4 | ~12V, high torque |
| Elbow Motor | Pololu 4756 Gearmotor | 2 | 5, 6 | Built-in 64 PPR encoder, 131:1 gear |
| Wheel Motor | JGB37-520 DC Motor | 2 | 1, 2 | 12V, differential drive |
| Motor Driver | IBT-2 (BTS7960) | 4 | 3,4,5,6 | High-current H-bridge |
| Motor Driver | L298N | 2 | 1, 2 | Dual H-bridge + 5V supply |
| Encoder | YT06-OP Rotary 600 PPR | 4 | 1,2,3,4 | External, quadrature |
| Battery | 12V 1.3Ah SLA | 1 | — | Power source + counterweight |

**Total hardware cost:** ~570 USD

---

## PID Parameters

| Motor | Joint | Kp | Ki | Kd | Loop rate | Encoder counts/rev |
|---|---|---|---|---|---|---|
| JGB37-520 | Wheels | 30 | 2 | 0 | 200 Hz | 600 (×1 mode) |
| Keskinler Wiper | Shoulders | 0.8 | 0.02 | 0.000001 | 1000 Hz | 2400 (600 PPR ×4) |
| Pololu 4756 | Elbows | 40 | 10 | 2 | 100 Hz | 8384 (64 PPR × 131) |

All gains are tunable at runtime over WiFi without reflashing firmware.

---

## Wiring & Pin Assignments

### STM32F407 — Shoulder (Keskinler, TIM1 encoder + TIM2 PWM)

| Signal | Pin | Timer / Peripheral |
|---|---|---|
| Encoder A | PA8 | TIM1 CH1 |
| Encoder B | PE11 | TIM1 CH2 |
| PWM Forward | PA0 | TIM2 CH1 |
| PWM Reverse | PA1 | TIM2 CH2 |
| CAN TX | PD1 | CAN1 TX |
| CAN RX | PD0 | CAN1 RX |

### STM32F407 — Wheel (JGB37, TIM3 encoder + TIM4 PWM)

| Signal | Pin | Timer / Peripheral |
|---|---|---|
| Encoder A | PC6 | TIM3 CH1 |
| Encoder B | PC7 | TIM3 CH2 |
| PWM ENA | PB7 | TIM4 CH2 |
| Motor INA | PB14 | GPIO output |
| Motor INB | PB15 | GPIO output |
| CAN TX | PD1 | CAN1 TX |
| CAN RX | PD0 | CAN1 RX |

### STM32F407 — Elbow (Pololu, TIM3 encoder + TIM1 PWM)

| Signal | Pin | Timer / Peripheral |
|---|---|---|
| Encoder A | PC6 | TIM3 CH1 |
| Encoder B | PC7 | TIM3 CH2 |
| PWM Forward | PA8 | TIM1 CH1 |
| PWM Reverse | PA9 | TIM1 CH2 |
| CAN TX | PD1 | CAN1 TX |
| CAN RX | PD0 | CAN1 RX |

### ESP32

| Signal | GPIO |
|---|---|
| CAN TX (to SN65HVD230 TXD) | GPIO 4 |
| CAN RX (from SN65HVD230 RXD) | GPIO 15 |

---

## Getting Started

### 1. Flash the STM32 nodes

Open each project in **STM32CubeIDE**. Before flashing, set `MOTOR_ID` to the correct value (1–6) at the top of `main.c`:

```c
#define MOTOR_ID   1   // Change per node
```

Flash all six boards. Each node will start waiting for CAN commands immediately after power-on.

### 2. Flash the ESP32

Open `embedded/esp32/main.ino` in Arduino IDE with the ESP32 board package installed. Flash normally. The ESP32 creates the WiFi AP and starts the HTTP server automatically.

### 3. Verify the CAN bus

Connect to `ESP32_Motor_Net` (password: `password123`), then open a browser or run:

```bash
curl http://192.168.4.1/ping
```

Expected output:
```
CAN State : 0 (0=RUNNING, 2=BUS-OFF)
TX Error  : 0 (128+ hat kopuk)
RX Error  : 0
```

If `State` is 2 (BUS-OFF) or TX errors are above 128, check termination resistors and transceiver wiring.

### 4. Run the Python interface

```bash
pip install requests
python host_interface/motor_controller.py
```

Send a test position command:
```
motor5
pos 45
```

Watch the fetch thread output — `CurPos` should converge toward `45.00°`.

---

## Repository Structure

```
embedded/
├── shoulder/           # STM32 firmware — Keskinler wiper motor (Motor IDs 3, 4)
│   └── main.c          # TIM1 encoder · TIM2 PWM · 1 kHz PID · IBT-2 driver
├── wheel/              # STM32 firmware — JGB37 wheel motor (Motor IDs 1, 2)
│   └── main.c          # TIM3 encoder · TIM4 PWM · 200 Hz PID · L298N driver
├── elbow/              # STM32 firmware — Pololu gearmotor (Motor IDs 5, 6)
│   └── main.c          # TIM3 encoder · TIM1 PWM · 100 Hz PID · IBT-2 driver
└── esp32/
    └── main.ino        # WiFi AP · HTTP server · TWAI CAN · motor profiles

host_interface/
└── motor_controller.py # Python CLI · threaded fetch · /set and /status endpoints
```
