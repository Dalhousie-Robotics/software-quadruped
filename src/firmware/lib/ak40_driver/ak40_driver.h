/**
 * ak40_driver.h -- CubeMars AK40-10 motor driver (MIT Mini Cheetah protocol)
 *
 * Targets STM32F446RE with STM32CubeIDE / HAL.
 * hcan1 is declared in the CubeIDE-generated main.h -- include that before
 * including this file, or pass the handle explicitly to each function.
 *
 * Pin assignments:
 *   CAN1 RX: PA11
 *   CAN1 TX: PA12
 */

#pragma once

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* -------------------------------------------------------------------------
 * Parameter ranges (AK40-10 MIT mode)
 * ---------------------------------------------------------------------- */
#define AK40_P_MIN   -12.5f   /* rad */
#define AK40_P_MAX    12.5f
#define AK40_V_MIN   -65.0f   /* rad/s */
#define AK40_V_MAX    65.0f
#define AK40_KP_MIN    0.0f   /* Nm/rad */
#define AK40_KP_MAX  500.0f
#define AK40_KD_MIN    0.0f   /* Nm-s/rad */
#define AK40_KD_MAX    5.0f
#define AK40_T_MIN   -18.0f   /* Nm */
#define AK40_T_MAX    18.0f

/* -------------------------------------------------------------------------
 * Motor state returned from send_command
 * ---------------------------------------------------------------------- */
typedef struct {
    uint8_t id;
    float   position;   /* rad */
    float   velocity;   /* rad/s */
    float   torque;     /* Nm */
} AK40_State;

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * Configure the CAN1 RX filter and start the peripheral.
 * Call once in main() after MX_CAN1_Init().
 */
HAL_StatusTypeDef AK40_Init(CAN_HandleTypeDef *hcan);

/** Send enter-MIT-mode frame. Must be called before send_command. */
void AK40_EnterMode(CAN_HandleTypeDef *hcan, uint8_t id);

/** Send exit-MIT-mode frame. Call on shutdown or emergency stop. */
void AK40_ExitMode(CAN_HandleTypeDef *hcan, uint8_t id);

/** Set zero position (writes to motor flash -- use carefully). */
void AK40_SetZero(CAN_HandleTypeDef *hcan, uint8_t id);

/**
 * Send a MIT-mode command and poll for the motor reply.
 * Returns HAL_OK if a reply was received within timeout_ms, HAL_TIMEOUT otherwise.
 * out may be NULL if feedback is not needed.
 */
HAL_StatusTypeDef AK40_SendCommand(CAN_HandleTypeDef *hcan,
                                   uint8_t id,
                                   float pos, float vel,
                                   float kp,  float kd,
                                   float torque_ff,
                                   AK40_State *out,
                                   uint32_t timeout_ms);
