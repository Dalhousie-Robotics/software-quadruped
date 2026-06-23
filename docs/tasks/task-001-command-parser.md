# Task 001: Command Parser

**Phase:** 1 -- Single Motor Control
**Difficulty:** Beginner / intermediate
**Language:** C
**Toolchain:** STM32CubeIDE
**Estimated time:** 4-8 hours
**Good first issue:** Yes

---

## What you are building

A terminal-driven firmware program that runs on the STM32 Nucleo F446RE. You type a command into a serial terminal on your PC, one motor responds. The point is to verify each motor works correctly in isolation before anything more complex is attempted.

This is the simplest possible test of the full hardware chain:

```
PC terminal  ->  USART2  ->  STM32  ->  CAN bus  ->  AK40-10 motor
                                  <-                <-
```

---

## Hardware

| Component | Detail |
|---|---|
| Microcontroller | STM32F446RE (Nucleo F446RE board) |
| Motors | CubeMars AK40-10, CAN IDs 1, 2, 3 |
| CAN transceiver | Required between STM32 and motors (e.g. SN65HVD230 or TJA1050) |
| CAN bus speed | 1 Mbit/s |
| Serial to PC | USART2 via ST-Link virtual COM port |

**Pin assignments (STM32F446RE):**

| Pin | Function |
|---|---|
| PA2 | USART2 TX (to ST-Link, appears as virtual COM on PC) |
| PA3 | USART2 RX |
| PA11 | CAN1 RX (connect to transceiver CRXD) |
| PA12 | CAN1 TX (connect to transceiver CTXD) |

> Note: PA11/PA12 are free when using USART2 for serial (not USB CDC). Do not enable USB OTG in CubeMX if using these pins for CAN.

---

## Setting up the project in STM32CubeIDE

1. New STM32 Project -> Board Selector -> search `NUCLEO-F446RE` -> Finish
2. In CubeMX (the .ioc file):
   - Enable **USART2**: Asynchronous, 115200 baud, PA2/PA3
   - Enable **CAN1**: PA11/PA12, Prescaler set for 1 Mbit/s at 180 MHz clock
   - Enable **NVIC**: CAN1 RX0 interrupt (for receiving motor replies)
3. Generate code
4. Write your application in `Core/Src/main.c`

**CAN timing for 1 Mbit/s at 180 MHz APB1 (90 MHz):**
```
Prescaler = 9
Time Seg 1 = 6
Time Seg 2 = 3
SJW = 1
```
This gives: 90 MHz / 9 / (1 + 6 + 3) = 1 MHz. Verify with an online CAN bit timing calculator if unsure.

---

## Commands to implement

Parse one line at a time (terminated by `\n` or `\r\n`) from USART2:

| Command | Action |
|---|---|
| `enable <id>` | Send enter-MIT-mode frame to motor `id` |
| `disable <id>` | Send exit-MIT-mode frame to motor `id` |
| `zero <id>` | Set current position as zero on motor `id` |
| `pos <id> <angle>` | Move motor `id` to `angle` radians |
| `stop` | Disable all motors (send exit-mode to IDs 1, 2, 3) |
| `status` | Print last received feedback from all active motors |

Print a response to USART2 for every command so the user knows it worked.

---

## The CAN message format (MIT mode)

The AK40-10 uses the MIT Mini Cheetah actuator protocol. Each CAN frame is 8 bytes.

### Command frame (STM32 -> motor, CAN ID = motor ID)

```
Bytes [0:1]   position target    uint16, maps -12.5 to +12.5 rad
Bytes [2:3]   velocity target    uint12, maps -65.0 to +65.0 rad/s
Bytes [3:4]   kp                 uint12, maps 0 to 500 Nm/rad
Bytes [5:6]   kd                 uint12, maps 0 to 5 Nm-s/rad
Bytes [6:7]   feedforward torque uint12, maps -18 to +18 Nm
```

Exact bit packing:
```c
data[0] = p_int >> 8;
data[1] = p_int & 0xFF;
data[2] = v_int >> 4;
data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
data[4] = kp_int & 0xFF;
data[5] = kd_int >> 4;
data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
data[7] = t_int & 0xFF;
```

### Response frame (motor -> STM32)

```
Byte  [0]     Motor ID
Bytes [1:2]   position     uint16
Bytes [3:4]   velocity     uint12 (upper 8 bits in [3], lower 4 in [4] upper nibble)
Bytes [4:5]   torque       uint12 (lower 4 of [4] are upper bits)
Bytes [6:7]   temperature and error flags
```

### Special frames (all 8 bytes, to motor CAN ID)

```c
// Enter MIT mode
uint8_t enter[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
// Exit MIT mode
uint8_t exit[]  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
// Set zero position
uint8_t zero[]  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
```

### Float to uint conversion

```c
uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (uint16_t)((x - x_min) / span * ((1 << bits) - 1));
}
```

---

## Existing work to reference

The full MIT protocol is already implemented in C++ for PlatformIO. You can use it as a direct reference -- the bit packing is identical, only the HAL CAN API calls differ:

- `src/firmware/lib/ak40_driver/ak40_driver.cpp` -- encoding/decoding, special frames
- `src/firmware/lib/ak40_driver/ak40_driver.h` -- parameter ranges (P_MIN, P_MAX, etc.)
- `docs/architecture.md` -- full protocol spec with byte-by-byte diagram

The difference: our PlatformIO version calls the STM32duino CAN library. Your CubeIDE version will call `HAL_CAN_AddTxMessage()` and `HAL_CAN_GetRxMessage()` directly. The logic is the same.

---

## Suggested gains for `pos` command

Safe starting values for a benchtop test with the motor clamped down:

```c
float kp = 5.0f;   // Nm/rad -- low, safe for first test
float kd = 0.5f;   // Nm-s/rad
float vel = 0.0f;
float torque_ff = 0.0f;
```

Increase kp gradually once you confirm the motor responds.

---

## Done when

- [ ] Project builds cleanly in STM32CubeIDE
- [ ] `enable 1` puts motor 1 into MIT mode (motor becomes stiff)
- [ ] `pos 1 0.5` moves motor 1 to 0.5 rad
- [ ] Serial terminal prints motor feedback (position, velocity, torque)
- [ ] `disable 1` releases motor (goes limp)
- [ ] `stop` disables all three motors

---

## Tips

- Configure motor IDs with the R-Link tool before starting. Default is ID 1.
- The CAN bus needs a 120-ohm termination resistor at each end.
- If the motor does not respond, check the transceiver wiring and confirm the bus speed matches.
- `HAL_CAN_Start()` must be called before any transmit/receive.
- Use `HAL_UART_Transmit()` for debug prints while setting up, then switch to interrupt-driven once it works.
