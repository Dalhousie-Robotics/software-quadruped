# Dal Robotics Quadruped Robot

A campus-autonomous quadruped robot with AI-driven student interaction, built by the Dal Robotics club at Dalhousie University.

---

## Quick Links

| Resource | Link |
|---|---|
| Task Board | https://github.com/orgs/Dalhousie-Robotics/projects/1 |
| Onboarding (start here) | https://github.com/Dalhousie-Robotics/software-onboarding |
| Team Roadmap | https://github.com/Dalhousie-Robotics/software-docs/blob/main/roadmap.md |
| Architecture Overview | [docs/architecture.md](docs/architecture.md) |
| Contributing Guide | [CONTRIBUTING.md](CONTRIBUTING.md) |
| Meeting Notes | https://github.com/Dalhousie-Robotics/software-docs/tree/main/meetings |
| ML Interface Agreement | https://github.com/Dalhousie-Robotics/software-docs/blob/main/ml_interface_agreement.md |

---

## What We're Building

A 12-DOF quadruped robot (3 joints per leg) inspired by MIT Cheetah and Boston Dynamics Spot. The robot navigates Dalhousie's campus autonomously and interacts with students using onboard AI.

**Subsystems:**

| Subsystem | Description |
|---|---|
| Mechanical | Frame, leg linkages, body structure |
| Electrical | Power distribution, wiring, CAN transceiver circuits |
| Software & Simulations | Motor firmware, gait control, simulation |
| Machine Learning | Person detection, navigation AI, conversation |

---

## System Architecture

```
Windows PC (Python)
  -- Gait planner, kinematics, path commands
  -- MuJoCo simulation environment
        |
        |  USB Serial (dev) / WiFi UDP (Phase 5+)
        v
STM32F446RE Microcontroller (C++ firmware)
  -- Real-time CAN control loop @ 500-1000 Hz
  -- Safety watchdog, command parsing
        |
        |  CAN bus @ 1 Mbit/s
        v
AK40-10 Motors x 12 (CubeMars)
  -- MIT Mini Cheetah CAN protocol
```

---

## Repository Structure

```
quadruped/
+-- src/
|   +-- firmware/          STM32 firmware (PlatformIO / C++)
|   |   +-- src/           Main firmware source
|   |   +-- lib/           Motor driver library (ak40_driver)
|   +-- control/           High-level Python control (PC-side)
|   +-- simulation/        MuJoCo simulation models and scripts
+-- docs/
|   +-- architecture.md    System architecture deep-dive
|   +-- decisions/         Architecture Decision Records (ADRs)
+-- tests/                 Unit and integration tests
+-- scripts/               Utility scripts (motor_tuner.py, calibration)
+-- .github/
    +-- workflows/         CI pipelines
    +-- ISSUE_TEMPLATE/    Bug and feature request templates
```

---

## Tech Stack

| Layer | Technology |
|---|---|
| STM32 Firmware | C++ via PlatformIO (stm32duino Arduino framework) |
| Microcontroller | STM32F446RE (Nucleo F446RE board) |
| CAN Communication | STM32 bxCAN peripheral + MIT Mini Cheetah protocol |
| High-Level Control | Python 3.10+ |
| Simulation | MuJoCo 3.x (Windows) |
| Motor Configuration | R-Link software (CubeMars, Windows) |
| PC to STM32 Comms | USB Serial (dev) / WiFi UDP (Phase 5+) |
| CI/CD | GitHub Actions |

---

## Getting Started

1. Follow the [Onboarding Guide](https://github.com/Dalhousie-Robotics/software-onboarding) -- do not skip this.
2. Go to the [Task Board](https://github.com/orgs/Dalhousie-Robotics/projects/1) and pick up an issue tagged `good-first-issue`.
3. Read [CONTRIBUTING.md](CONTRIBUTING.md) before opening your first PR.

---

## Team

### Firmware
*Writes C++ code that runs on the STM32, handling CAN bus communication with motors in real time.*

| Name | GitHub | Current Task |
|---|---|---|
| | | |
| | | |

### Simulation
*Builds and maintains the MuJoCo robot model, letting the team test algorithms without touching hardware.*

| Name | GitHub | Current Task |
|---|---|---|
| | | |

### Kinematics & Control
*Writes Python algorithms for leg movement: forward/inverse kinematics, gait planning, foot trajectories.*

| Name | GitHub | Current Task |
|---|---|---|
| | | |
| | | |

### Integration & Lead
*Maintains repo health, reviews all PRs, coordinates between areas, owns the architecture.*

| Name | GitHub | Current Task |
|---|---|---|
| | | Software Lead |

*Fill in names and tasks when roles are assigned at the first meeting.*

---

## Current Status

**Phase 0: Infrastructure** (active)

See the full [roadmap](https://github.com/Dalhousie-Robotics/software-docs/blob/main/roadmap.md) for phase breakdown.
