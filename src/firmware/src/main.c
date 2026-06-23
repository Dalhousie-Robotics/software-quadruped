/**
 * main.c -- Dal Robotics Quadruped Firmware
 * STM32F446RE (Nucleo), STM32CubeIDE / HAL / C
 *
 * HOW TO USE IN CUBEIDE:
 *   1. Create a new STM32 project: Board Selector -> NUCLEO-F446RE
 *   2. In CubeMX (.ioc):
 *        USART2  Asynchronous, 115200 baud, PA2 TX / PA3 RX
 *        CAN1    PA11 RX / PA12 TX, prescaler for 1 Mbit/s (see below)
 *        NVIC    Enable USART2 global interrupt
 *   3. Generate code
 *   4. Copy ak40_driver.h into Core/Inc/ and ak40_driver.c into Core/Src/
 *   5. Paste the USER CODE sections from this file into the generated main.c
 *
 * CAN timing for 1 Mbit/s at 90 MHz APB1:
 *   Prescaler=9, TimeSeg1=6, TimeSeg2=3, SJW=1
 *
 * Serial pins: PA2 (TX), PA3 (RX) via ST-Link virtual COM port, 115200 baud.
 * CAN pins:    PA11 (RX), PA12 (TX) -- connect to CAN transceiver.
 *
 * Command protocol (newline-terminated ASCII):
 *   enable <id>          Enter MIT mode on motor id
 *   disable <id>         Exit MIT mode
 *   zero <id>            Set zero position
 *   pos <id> <rad>       Move motor to position (radians)
 *   stop                 Disable all motors (IDs 1-3)
 *   status               Print last received motor state
 */

/* USER CODE BEGIN Includes */
#include "ak40_driver.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* These handles are declared by CubeIDE-generated code: */
extern CAN_HandleTypeDef  hcan1;
extern UART_HandleTypeDef huart2;

/* -------------------------------------------------------------------------
 * USER CODE BEGIN PD
 * ---------------------------------------------------------------------- */
#define RX_BUF_LEN   64
#define NUM_MOTORS    3
/* USER CODE END PD */

/* -------------------------------------------------------------------------
 * USER CODE BEGIN PV
 * ---------------------------------------------------------------------- */
static char     rx_line[RX_BUF_LEN];
static uint8_t  rx_char;
static uint8_t  rx_idx = 0;
static uint8_t  line_ready = 0;

static AK40_State last_state[NUM_MOTORS + 1];  /* index by motor ID 1..3 */
/* USER CODE END PV */

/* -------------------------------------------------------------------------
 * USER CODE BEGIN 0  (helper functions)
 * ---------------------------------------------------------------------- */
static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

static void uart_printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    uart_print(buf);
}

static void process_command(char *line)
{
    /* Trim trailing whitespace */
    int len = (int)strlen(line);
    while (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n' || line[len-1] == ' '))
        line[--len] = '\0';

    if (len == 0) return;

    /* --- enable <id> --- */
    if (strncmp(line, "enable ", 7) == 0) {
        uint8_t id = (uint8_t)atoi(line + 7);
        AK40_EnterMode(&hcan1, id);
        uart_printf("[ok] enter mode -> motor %d\r\n", id);

    /* --- disable <id> --- */
    } else if (strncmp(line, "disable ", 8) == 0) {
        uint8_t id = (uint8_t)atoi(line + 8);
        AK40_ExitMode(&hcan1, id);
        uart_printf("[ok] exit mode -> motor %d\r\n", id);

    /* --- zero <id> --- */
    } else if (strncmp(line, "zero ", 5) == 0) {
        uint8_t id = (uint8_t)atoi(line + 5);
        AK40_SetZero(&hcan1, id);
        uart_printf("[ok] zero set -> motor %d\r\n", id);

    /* --- pos <id> <rad> --- */
    } else if (strncmp(line, "pos ", 4) == 0) {
        uint8_t id;
        float   angle;
        if (sscanf(line + 4, "%hhu %f", &id, &angle) == 2) {
            AK40_State st = {0};
            HAL_StatusTypeDef r = AK40_SendCommand(&hcan1, id,
                                                    angle, 0.0f,
                                                    5.0f,  0.5f,
                                                    0.0f,  &st, 10);
            if (r == HAL_OK) {
                if (id <= NUM_MOTORS) last_state[id] = st;
                uart_printf("STATE %d %.4f %.4f %.4f\r\n",
                            st.id, st.position, st.velocity, st.torque);
            } else {
                uart_printf("[error] motor %d no response\r\n", id);
            }
        } else {
            uart_print("[error] usage: pos <id> <rad>\r\n");
        }

    /* --- stop --- */
    } else if (strcmp(line, "stop") == 0) {
        for (uint8_t i = 1; i <= NUM_MOTORS; i++) {
            AK40_ExitMode(&hcan1, i);
        }
        uart_print("[ok] all motors disabled\r\n");

    /* --- status --- */
    } else if (strcmp(line, "status") == 0) {
        for (uint8_t i = 1; i <= NUM_MOTORS; i++) {
            AK40_State *s = &last_state[i];
            uart_printf("motor %d: pos=%.4f rad  vel=%.4f rad/s  torque=%.4f Nm\r\n",
                        i, s->position, s->velocity, s->torque);
        }

    } else {
        uart_printf("[error] unknown command: %s\r\n", line);
    }
}
/* USER CODE END 0 */


/* -------------------------------------------------------------------------
 * USER CODE BEGIN 2  (put after all MX_Init calls in main())
 * ---------------------------------------------------------------------- */
/*
    uart_print("\r\n[boot] Dal Robotics Quadruped Firmware\r\n");
    uart_print("[boot] STM32F446RE  CAN1 PA11/PA12  USART2 PA2/PA3\r\n");

    if (AK40_Init(&hcan1) != HAL_OK) {
        uart_print("[error] CAN init failed -- check transceiver wiring\r\n");
        Error_Handler();
    }
    uart_print("[ok] CAN bus ready at 1 Mbit/s\r\n");
    uart_print("[ok] Waiting for commands...\r\n");

    // Start interrupt-driven UART receive
    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
*/
/* USER CODE END 2 */


/* -------------------------------------------------------------------------
 * USER CODE BEGIN WHILE  (inside the while(1) loop in main())
 * ---------------------------------------------------------------------- */
/*
    if (line_ready) {
        line_ready = 0;
        process_command(rx_line);
        memset(rx_line, 0, sizeof(rx_line));
        rx_idx = 0;
    }
*/
/* USER CODE END WHILE */


/* -------------------------------------------------------------------------
 * USER CODE BEGIN 4  (UART receive callback -- add to stm32f4xx_it.c or here)
 * ---------------------------------------------------------------------- */
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        if (rx_char == '\n' || rx_char == '\r') {
            if (rx_idx > 0) line_ready = 1;
        } else if (rx_idx < RX_BUF_LEN - 1) {
            rx_line[rx_idx++] = (char)rx_char;
            rx_line[rx_idx]   = '\0';
        }
        HAL_UART_Receive_IT(&huart2, &rx_char, 1);
    }
}
*/
/* USER CODE END 4 */
