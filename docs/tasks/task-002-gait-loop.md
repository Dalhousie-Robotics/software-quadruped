# Task 002: Gait Loop

**Phase:** 1 -- Single Motor Control (single leg demo)
**Difficulty:** Intermediate / advanced
**Language:** C or C++
**Toolchain:** STM32CubeIDE
**Estimated time:** 8-16 hours
**Pair recommended:** Yes -- the IK section is fiddly

---

## What you are building

A real-time gait controller for a single leg (3 motors: hip, thigh, knee). Every millisecond, the program computes where the foot should be in that moment of the walking cycle, solves for the joint angles needed to reach that point, and sends CAN commands to all three motors simultaneously. On repeat, 1000 times per second. That synchronised update is what produces smooth, coordinated leg motion instead of three motors twitching independently.

This is not a simulation. This runs on the STM32 and drives real hardware.

---

## Hardware

Same as Task 001. Three AK40-10 motors on the same CAN bus:

| Motor ID | Joint |
|---|---|
| 1 | Hip abduction/adduction |
| 2 | Hip flexion/extension (thigh) |
| 3 | Knee |

---

## File structure

Keep everything in one file to start. Put all tunable constants at the top:

```c
// ---- TUNE THESE BEFORE RUNNING ----
#define CYCLE_PERIOD_MS   1000   // ms per full step cycle
#define STEP_LENGTH_MM     80    // mm -- how far forward the foot travels
#define STEP_HEIGHT_MM     40    // mm -- peak foot lift during swing

// Link lengths -- update from CAD when available
#define L_THIGH_MM        200    // mm
#define L_SHANK_MM        200    // mm

// Gains -- start conservative
#define KP   5.0f   // Nm/rad
#define KD   0.5f   // Nm-s/rad
```

---

## State machine

The program must not move the moment it powers on. Implement at minimum:

```
IDLE      Waiting for start command. Motors enabled but holding position.
RUNNING   Gait loop active. Timer interrupt drives the trajectory.
STOPPED   Motors disabled (exit-mode sent). Safe to touch the leg.
```

Transitions:
- `start` command (serial) -> IDLE to RUNNING
- `stop` command -> any state to STOPPED
- On power-on -> IDLE

---

## 1 kHz timer interrupt

Use a STM32 hardware timer configured for a 1 ms interrupt. Do not use `HAL_Delay()` or `osDelay()` anywhere in the gait loop. The timer interrupt is the heartbeat.

In CubeMX: Timers -> TIM2 -> Clock Source: Internal Clock -> Counter Period: gives 1 ms at your APB1 clock -> Enable TIM2 global interrupt in NVIC.

In your interrupt handler:
```c
void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2 && state == RUNNING) {
        gait_tick();
    }
}
```

---

## Trajectory function

Given a phase value from 0.0 to 1.0 (where we are in the current step cycle), return the target foot position in 2D space (forward = x, up = z relative to the hip).

```
Phase 0.0 to 0.5: STANCE -- foot on ground, moving backward
Phase 0.5 to 1.0: SWING  -- foot in air, arcing forward
```

```c
typedef struct { float x; float z; } Vec2;

Vec2 foot_trajectory(float phase) {
    float half_step = STEP_LENGTH_MM / 2.0f;
    Vec2 foot;

    if (phase < 0.5f) {
        // Stance: foot moves linearly backward along the ground
        float t = phase / 0.5f;             // 0 -> 1 over stance
        foot.x = half_step - t * STEP_LENGTH_MM;  // +half -> -half
        foot.z = 0.0f;
    } else {
        // Swing: foot lifts and arcs forward using a half-sine for height
        float t = (phase - 0.5f) / 0.5f;   // 0 -> 1 over swing
        foot.x = -half_step + t * STEP_LENGTH_MM; // -half -> +half
        foot.z = STEP_HEIGHT_MM * sinf(t * M_PI); // 0 -> peak -> 0
    }
    return foot;
}
```

---

## Inverse kinematics

Given a target foot position (x, z) relative to the hip joint, compute the thigh and knee angles needed to place the foot there.

For a 2-link planar leg (thigh + shank, both in the sagittal plane):

```c
typedef struct { float hip_fe; float knee; } JointAngles;

JointAngles solve_ik(float foot_x, float foot_z) {
    float L1 = L_THIGH_MM / 1000.0f;  // m
    float L2 = L_SHANK_MM / 1000.0f;
    float x   = foot_x / 1000.0f;
    float z   = foot_z / 1000.0f;

    // Distance from hip to foot
    float d2  = x * x + z * z;
    float d   = sqrtf(d2);

    // Clamp to reachable workspace
    if (d > L1 + L2) d = L1 + L2 - 0.001f;
    if (d < fabsf(L1 - L2)) d = fabsf(L1 - L2) + 0.001f;

    // Knee angle (cosine rule)
    float cos_knee = (d2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    float knee = -acosf(cos_knee);  // negative = bent

    // Hip angle
    float alpha = atan2f(x, -z);    // angle to foot from vertical
    float beta  = acosf((d2 + L1 * L1 - L2 * L2) / (2.0f * d * L1));
    float hip_fe = alpha - beta;

    JointAngles angles = { hip_fe, knee };
    return angles;
}
```

> The hip abduction joint (motor 1) stays at 0 for a straight-ahead walk. Only hip_fe (motor 2) and knee (motor 3) are driven by IK.

---

## Main gait tick (runs every 1 ms)

```c
void gait_tick(void) {
    static uint32_t tick = 0;
    tick++;

    // Phase: 0.0 to 1.0, advances each tick
    float phase = fmodf((float)tick / CYCLE_PERIOD_MS, 1.0f);

    // Get foot target
    Vec2 foot = foot_trajectory(phase);

    // Solve IK
    JointAngles angles = solve_ik(foot.x, foot.z);

    // Send CAN commands to all 3 motors
    send_mit_command(1, 0.0f,         0.0f, KP, KD, 0.0f);  // hip_ab: hold zero
    send_mit_command(2, angles.hip_fe, 0.0f, KP, KD, 0.0f);  // hip_fe
    send_mit_command(3, angles.knee,   0.0f, KP, KD, 0.0f);  // knee
}
```

---

## Existing work to reference

- `src/firmware/lib/ak40_driver/ak40_driver.cpp` -- MIT CAN frame encoding
- `src/firmware/lib/ak40_driver/ak40_driver.h` -- parameter ranges
- `docs/architecture.md` -- full CAN protocol and motor ID table
- `docs/tasks/task-001-command-parser.md` -- CAN setup, HAL timing, pin assignments
- `src/simulation/trot_demo.py` -- the same trajectory + gait logic implemented in Python on the MuJoCo model. A useful reference for understanding the approach.

Complete Task 001 first, or at minimum have CAN transmit working before starting this task.

---

## Done when

- [ ] Timer interrupt fires at exactly 1 kHz (verify with oscilloscope or logic analyser if available)
- [ ] State machine works: power on -> IDLE, `start` -> RUNNING, `stop` -> STOPPED
- [ ] Leg cycles through a visible walking motion with foot lift and swing
- [ ] All 3 motors update every tick (no staggered updates)
- [ ] Tunable constants are all at the top of the file
- [ ] No `HAL_Delay()` calls anywhere in the control loop

---

## Tips

- Pair up on the IK. Get it working in Python first (`python -c "import math; ..."`) before putting it on the MCU -- much easier to debug.
- Clamp IK inputs to the reachable workspace or the `acosf` will return NaN and the motor will go to max position.
- Start with KP=5, KD=0.5. The motors will be gentle and you can increase once the trajectory looks correct.
- The thigh and shank lengths in the constants are placeholders. Update them from the Mechanical team's CAD before running on the real leg.
- Log phase, foot_x, foot_z, and joint angles over serial for the first few cycles so you can verify the trajectory before enabling the motors.
