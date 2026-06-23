# ADR-001: Use ESP32 with PlatformIO for Motor Control Firmware

**Date:** 2026-04-29
**Status:** Superseded by [ADR-001-stm32-platformio.md](ADR-001-stm32-platformio.md) (2026-05-03)
**Deciders:** Software Team Lead

---

## Context

We need a microcontroller to run the real-time CAN bus control loop for 12 AK40-10 motors. The controller must:
- Generate CAN frames at 500–1000 Hz without missing deadlines
- Support WiFi for wireless PC communication
- Be accessible to team members with varying experience levels
- Be affordable and available

## Decision

Use the **ESP32** (Espressif ESP32-S3 or classic ESP32) programmed with the **PlatformIO** toolchain inside VS Code.

## Rationale

| Option | Pros | Cons |
|---|---|---|
| ESP32 + PlatformIO | Built-in WiFi, built-in TWAI/CAN, large community, free, Arduino-compatible | 240 MHz (sufficient but not high-end real-time) |
| STM32 + CubeIDE | Excellent real-time, widely used in industry | More complex toolchain, no built-in WiFi, steeper learning curve |
| Raspberry Pi | Full Linux, easy Python | Not real-time, requires separate CAN hat, overkill for this layer |
| Arduino Mega | Simple | No WiFi, no CAN peripheral, too slow |

PlatformIO was chosen over Arduino IDE because it integrates with VS Code (the team's primary editor), supports proper dependency management via `library.json`, and has a CLI usable in CI.

## Consequences

- All firmware is written in C++ targeting the ESP32 Arduino framework.
- CAN communication uses the ESP32's built-in TWAI peripheral (no external CAN controller IC needed beyond a CAN transceiver, e.g., SN65HVD230).
- WiFi is available for deployed wireless comms without additional hardware.
- Team members writing firmware need basic C++ knowledge.
