/**
 * ak40_driver.c -- CubeMars AK40-10 motor driver implementation
 * STM32 HAL CAN (CubeIDE / C)
 */

#include "ak40_driver.h"
#include <string.h>

/* Special command frames */
static const uint8_t ENTER_MODE[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
static const uint8_t EXIT_MODE[8]  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
static const uint8_t SET_ZERO[8]   = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};

/* -------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------- */

static uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (uint16_t)((x - x_min) / (x_max - x_min) * (float)((1 << bits) - 1));
}

static float uint_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    return (float)x * (x_max - x_min) / (float)((1 << bits) - 1) + x_min;
}

static HAL_StatusTypeDef send_raw(CAN_HandleTypeDef *hcan,
                                   uint8_t id,
                                   const uint8_t *data)
{
    CAN_TxHeaderTypeDef hdr;
    uint32_t mailbox;

    hdr.StdId              = id;
    hdr.ExtId              = 0;
    hdr.IDE                = CAN_ID_STD;
    hdr.RTR                = CAN_RTR_DATA;
    hdr.DLC                = 8;
    hdr.TransmitGlobalTime = DISABLE;

    /* Wait for a free mailbox (up to 10 ms) */
    uint32_t t = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
        if (HAL_GetTick() - t > 10) return HAL_TIMEOUT;
    }

    return HAL_CAN_AddTxMessage(hcan, &hdr, (uint8_t *)data, &mailbox);
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

HAL_StatusTypeDef AK40_Init(CAN_HandleTypeDef *hcan)
{
    /* Accept all incoming CAN frames (mask = 0x0000) */
    CAN_FilterTypeDef filter;
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;

    HAL_StatusTypeDef s = HAL_CAN_ConfigFilter(hcan, &filter);
    if (s != HAL_OK) return s;

    return HAL_CAN_Start(hcan);
}

void AK40_EnterMode(CAN_HandleTypeDef *hcan, uint8_t id)
{
    send_raw(hcan, id, ENTER_MODE);
}

void AK40_ExitMode(CAN_HandleTypeDef *hcan, uint8_t id)
{
    send_raw(hcan, id, EXIT_MODE);
}

void AK40_SetZero(CAN_HandleTypeDef *hcan, uint8_t id)
{
    send_raw(hcan, id, SET_ZERO);
}

HAL_StatusTypeDef AK40_SendCommand(CAN_HandleTypeDef *hcan,
                                   uint8_t id,
                                   float pos, float vel,
                                   float kp,  float kd,
                                   float torque_ff,
                                   AK40_State *out,
                                   uint32_t timeout_ms)
{
    /* Encode MIT command frame */
    uint8_t data[8];
    uint16_t p   = float_to_uint(pos,       AK40_P_MIN,  AK40_P_MAX,  16);
    uint16_t v   = float_to_uint(vel,       AK40_V_MIN,  AK40_V_MAX,  12);
    uint16_t kp_ = float_to_uint(kp,        AK40_KP_MIN, AK40_KP_MAX, 12);
    uint16_t kd_ = float_to_uint(kd,        AK40_KD_MIN, AK40_KD_MAX, 12);
    uint16_t t   = float_to_uint(torque_ff, AK40_T_MIN,  AK40_T_MAX,  12);

    data[0] = p >> 8;
    data[1] = p & 0xFF;
    data[2] = v >> 4;
    data[3] = ((v & 0xF) << 4) | (kp_ >> 8);
    data[4] = kp_ & 0xFF;
    data[5] = kd_ >> 4;
    data[6] = ((kd_ & 0xF) << 4) | (t >> 8);
    data[7] = t & 0xFF;

    HAL_StatusTypeDef s = send_raw(hcan, id, data);
    if (s != HAL_OK) return s;

    if (out == NULL) return HAL_OK;

    /* Poll RX FIFO for reply */
    CAN_RxHeaderTypeDef rx_hdr;
    uint8_t rx_data[8];
    uint32_t deadline = HAL_GetTick() + timeout_ms;

    while (HAL_GetTick() < deadline) {
        if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
            if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_hdr, rx_data) == HAL_OK) {
                if (rx_hdr.DLC < 6) continue;
                out->id       = rx_data[0];
                uint16_t p_r  = ((uint16_t)rx_data[1] << 8) | rx_data[2];
                uint16_t v_r  = ((uint16_t)rx_data[3] << 4) | (rx_data[4] >> 4);
                uint16_t t_r  = (((uint16_t)rx_data[4] & 0xF) << 8) | rx_data[5];
                out->position = uint_to_float(p_r, AK40_P_MIN, AK40_P_MAX, 16);
                out->velocity = uint_to_float(v_r, AK40_V_MIN, AK40_V_MAX, 12);
                out->torque   = uint_to_float(t_r, AK40_T_MIN, AK40_T_MAX, 12);
                return HAL_OK;
            }
        }
    }
    return HAL_TIMEOUT;
}
