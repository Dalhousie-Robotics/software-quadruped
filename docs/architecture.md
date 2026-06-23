# System Architecture

This document describes the software and firmware architecture of the Dal Robotics quadruped. It is the authoritative reference for how the system is structured and why.

---

## Overview

The system is divided into two compute domains:

| Domain | Hardware | Language | Toolchain | Responsibility |
|---|---|---|---|---|
| On-robot (embedded) | STM32F446RE | C | STM32CubeIDE / HAL | Real-time motor control, CAN bus |
| Off-robot (PC) | Windows PC | Python | VS Code | Gait planning, simulation, high-level commands |

These two domains communicate over **USART2 via ST-Link virtual COM** during development. Wireless (WiFi via UART-attached module) is planned for Phase 5+.

---

## Layer Diagram

```
+------------------------------------------------------+
|  LAYER 5: Autonomy & AI (ML Team)                    |
|  Person detection, campus navigation, conversation   |
|  Interface: see docs/ml_interface_agreement.md       |
+------------------------------------------------------+
|  LAYER 4: High-Level Control (PC / Python)           |
|  Gait scheduler, foot trajectory planner,            |
|  body pose controller, command dispatcher            |
+------------------------------------------------------+
|  LAYER 3: Kinematics (PC / Python)                   |
|  Forward kinematics (FK), Inverse kinematics (IK)    |
|  for all 4 legs                                      |
+------------------------------------------------------+
|  LAYER 2: Serial Protocol (USART2 / WiFi UDP)        |
|  ASCII command packets: joint targets -> STM32       |
|  Joint states <- STM32                               |
+------------------------------------------------------+
|  LAYER 1: STM32F446RE Firmware (C / CubeIDE / HAL)  |
|  Receives joint targets, encodes MIT CAN frames,     |
|  reads back motor state, safety watchdog             |
+------------------------------------------------------+
|  LAYER 0: CAN Bus + Motors                           |
|  AK40-10 x 12, MIT Mini Cheetah CAN protocol        |
+------------------------------------------------------+
```

---

## STM32 Pin Assignments (Nucleo F446RE)

| Pin | Peripheral | Function |
|---|---|---|
| PA2 | USART2 TX | Serial to PC (via ST-Link virtual COM) |
| PA3 | USART2 RX | Serial from PC |
| PA11 | CAN1 RX | From CAN transceiver CRXD |
| PA12 | CAN1 TX | To CAN transceiver CTXD |

**CAN bus speed:** 1 Mbit/s
**USART2 baud rate:** 115200

**CAN timing for 1 Mbit/s at 90 MHz APB1:**
Prescaler=9, TimeSeg1=6, TimeSeg2=3, SJW=1

---

## Communication Protocol (PC to STM32)

ASCII commands over USART2, newline-terminated.

### Commands (PC -> STM32)

```
enable <id>
disable <id>
zero <id>
pos <id> <rad>
stop
status
```

### Response (STM32 -> PC)

```
STATE <id> <pos_rad> <vel_rad_s> <torque_Nm>
[ok] ...
[error] ...
```

---

## CAN Bus Architecture

Each AK40-10 motor has a unique CAN ID (1-12). The STM32F446RE uses its built-in **bxCAN** peripheral (CAN 2.0B). An external CAN transceiver (SN65HVD230 or TJA1050) is required between the STM32 and the CAN bus wires. The Electrical team provides this circuit.

Motor ID assignment:

| ID | Joint | Leg |
|---|---|---|
| 1 | Hip Abduction/Adduction | Front Left |
| 2 | Hip Flexion/Extension | Front Left |
| 3 | Knee | Front Left |
| 4 | Hip Abduction/Adduction | Front Right |
| 5 | Hip Flexion/Extension | Front Right |
| 6 | Knee | Front Right |
| 7 | Hip Abduction/Adduction | Rear Left |
| 8 | Hip Flexion/Extension | Rear Left |
| 9 | Knee | Rear Left |
| 10 | Hip Abduction/Adduction | Rear Right |
| 11 | Hip Flexion/Extension | Rear Right |
| 12 | Knee | Rear Right |

---

## MIT Mini Cheetah CAN Protocol

The AK40-10 uses the MIT Mini Cheetah actuator protocol. Each CAN frame is 8 bytes.

### Command frame (STM32 -> motor)

```
Byte [0:1]  position target     (uint16, maps to -12.5 to +12.5 rad)
Byte [2]    velocity [11:4]
Byte [3]    vel [3:0] | kp[11:8]
Byte [4]    kp [7:0]             (12-bit, maps to 0 to 500 Nm/rad)
Byte [5]    kd [11:4]
Byte [6]    kd [3:0] | t[11:8]   (kd maps to 0 to 5 Nm-s/rad)
Byte [7]    torque_ff [7:0]      (12-bit, maps to -18 to +18 Nm)
```

### Response frame (motor -> STM32)

```
Byte [0]    Motor ID
Byte [1:2]  position
Byte [3:4]  velocity (12-bit packed)
Byte [4:5]  torque   (12-bit packed)
Byte [6:7]  temperature and error flags
```

### Special frames (to motor CAN ID)

| Frame | Last byte | Purpose |
|---|---|---|
| `FF FF FF FF FF FF FF FC` | 0xFC | Enter MIT mode |
| `FF FF FF FF FF FF FF FD` | 0xFD | Exit MIT mode |
| `FF FF FF FF FF FF FF FE` | 0xFE | Set zero position |

---

## Simulation Architecture

The simulation mirrors the real system using the same Python control code. A `SimMotorInterface` class exposes the same API as the real `SerialMotorInterface`, so all gait and kinematics code is hardware-agnostic.

```
src/control/gait_controller.py
    uses: MotorInterface (abstract)
          -- SerialMotorInterface  (real STM32 path via USART2)
          -- SimMotorInterface     (MuJoCo path)
```

---

## Key Design Decisions

| ADR | Decision |
|---|---|
| ADR-001 | Use STM32F446RE + STM32CubeIDE + HAL C for motor control firmware |
| ADR-002 | Use MuJoCo for simulation |
| ADR-003 | Wire protocol between PC and STM32 (TBD) |
