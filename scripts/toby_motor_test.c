// toby_motor_test.c
// just testing each AK40-10 motor one at a time over serial
// type a command, motor moves, see if it works before touching anything real
//
// setup in main.c:
//   1. paste the CAN filter into MX_CAN1_Init() after HAL_CAN_Init() (bottom of this file)
//   2. USER CODE BEGIN 2:  HAL_CAN_Start(&hcan1);  init(&hcan1, &huart2);
//   3. while loop:         process();
//   4. add callbacks:
//        void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) { can_rx(hcan); }
//        void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { if (huart->Instance == USART2) uart_rx(); }

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// AK40-10 limits from datasheet
#define P_MIN  -12.5f
#define P_MAX   12.5f
#define V_MIN  -65.0f
#define V_MAX   65.0f
#define T_MIN  -65.0f
#define T_MAX   65.0f
#define KP_MAX  500.0f
#define KD_MAX    5.0f

typedef struct { float pos, vel, torque; uint8_t got_data; } Feedback;
static Feedback fb[3];

static CAN_HandleTypeDef  *can;
static UART_HandleTypeDef *uart;

static uint8_t rx_byte;
static char    line_buf[64];
static uint8_t line_pos   = 0;
static uint8_t line_ready = 0;


// float <-> N-bit uint, needed to pack the MIT CAN frame
static uint16_t f2u(float x, float mn, float mx, int bits)
{
    float t = (x - mn) / (mx - mn);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return (uint16_t)(t * ((1 << bits) - 1));
}

static float u2f(uint16_t raw, float mn, float mx, int bits)
{
    return mn + (float)raw / (float)((1 << bits) - 1) * (mx - mn);
}

static void can_send(uint8_t id, uint8_t d[8])
{
    CAN_TxHeaderTypeDef h = {0};
    h.StdId = id; h.IDE = CAN_ID_STD; h.RTR = CAN_RTR_DATA; h.DLC = 8;
    uint32_t mailbox;
    HAL_CAN_AddTxMessage(can, &h, d, &mailbox);
}

static void say(const char *s)
{
    HAL_UART_Transmit(uart, (uint8_t *)s, strlen(s), 100);
}


// MIT frame: pack pos/vel/kp/kd/torque into 8 bytes and send
// layout: [pos 16bit][vel 12bit][kp 12bit][kd 12bit][torque 12bit]
static void mit_cmd(uint8_t id, float pos, float vel, float kp, float kd, float tor)
{
    uint16_t p = f2u(pos, P_MIN, P_MAX, 16);
    uint16_t v = f2u(vel, V_MIN, V_MAX, 12);
    uint16_t k = f2u(kp,  0, KP_MAX, 12);
    uint16_t d_ = f2u(kd, 0, KD_MAX, 12);
    uint16_t t = f2u(tor, T_MIN, T_MAX, 12);

    uint8_t d[8];
    d[0] = p >> 8;
    d[1] = p & 0xFF;
    d[2] = v >> 4;
    d[3] = ((v & 0xF) << 4) | (k >> 8);
    d[4] = k & 0xFF;
    d[5] = d_ >> 4;
    d[6] = ((d_ & 0xF) << 4) | (t >> 8);
    d[7] = t & 0xFF;

    can_send(id, d);
}


// magic bytes from the CubeMars datasheet for enter/exit/zero
void motor_enable(uint8_t id)
{
    uint8_t d[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
    can_send(id, d);
    char buf[32]; snprintf(buf, sizeof(buf), "motor %u on\r\n> ", id); say(buf);
}

void motor_disable(uint8_t id)
{
    uint8_t d[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
    can_send(id, d);
    char buf[32]; snprintf(buf, sizeof(buf), "motor %u off\r\n> ", id); say(buf);
}

void motor_zero(uint8_t id)
{
    uint8_t d[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
    can_send(id, d);
    char buf[32]; snprintf(buf, sizeof(buf), "motor %u zeroed\r\n> ", id); say(buf);
}

void motor_pos(uint8_t id, float rad)
{
    mit_cmd(id, rad, 0, 25.0f, 1.0f, 0);  // kp=25 kd=1, tune if needed
    char buf[40]; snprintf(buf, sizeof(buf), "motor %u -> %.3f rad\r\n> ", id, (double)rad); say(buf);
}

void stop_all(void)
{
    motor_disable(1); motor_disable(2); motor_disable(3);
    say("all stopped\r\n> ");
}

void print_status(void)
{
    char buf[80];
    for (int i = 0; i < 3; i++) {
        if (fb[i].got_data)
            snprintf(buf, sizeof(buf), "motor %d  pos %.3f  vel %.3f  torque %.3f\r\n",
                i+1, (double)fb[i].pos, (double)fb[i].vel, (double)fb[i].torque);
        else
            snprintf(buf, sizeof(buf), "motor %d  no data\r\n", i+1);
        say(buf);
    }
    say("> ");
}


// called from HAL_CAN_RxFifo0MsgPendingCallback
// reply frame: [id][pos 16bit][vel 12bit][torque 12bit]
void can_rx(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef h;
    uint8_t d[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &h, d) != HAL_OK) return;
    if (h.DLC < 6) return;

    uint8_t id = d[0];
    if (id < 1 || id > 3) return;

    fb[id-1].pos      = u2f(((uint16_t)d[1] << 8) | d[2],          P_MIN, P_MAX, 16);
    fb[id-1].vel      = u2f(((uint16_t)d[3] << 4) | (d[4] >> 4),   V_MIN, V_MAX, 12);
    fb[id-1].torque   = u2f(((uint16_t)(d[4] & 0xF) << 8) | d[5],  T_MIN, T_MAX, 12);
    fb[id-1].got_data = 1;
}


// called from HAL_UART_RxCpltCallback, collects chars until enter
void uart_rx(void)
{
    uint8_t b = rx_byte;
    HAL_UART_Transmit(uart, &b, 1, 10);

    if (b == '\r' || b == '\n') {
        if (line_pos > 0) { line_buf[line_pos] = '\0'; line_pos = 0; line_ready = 1; }
        say("\r\n");
    } else if (b == 0x7F || b == 0x08) {
        if (line_pos > 0) { line_pos--; say(" \b"); }
    } else {
        if (line_pos < 63) line_buf[line_pos++] = b;
    }

    HAL_UART_Receive_IT(uart, &rx_byte, 1);
}


// call in while(1)
void process(void)
{
    if (!line_ready) return;
    line_ready = 0;

    char buf[64];
    strncpy(buf, line_buf, sizeof(buf));
    char *cmd = strtok(buf, " ");
    if (!cmd) { say("> "); return; }

    if (strcmp(cmd, "stop")   == 0) { stop_all();     return; }
    if (strcmp(cmd, "status") == 0) { print_status();  return; }

    char *arg1 = strtok(NULL, " ");
    if (!arg1) { say("need an id\r\n> "); return; }
    uint8_t id = (uint8_t)atoi(arg1);
    if (id < 1 || id > 3) { say("id 1-3 only\r\n> "); return; }

    if (strcmp(cmd, "enable")  == 0) { motor_enable(id);  return; }
    if (strcmp(cmd, "disable") == 0) { motor_disable(id); return; }
    if (strcmp(cmd, "zero")    == 0) { motor_zero(id);    return; }

    if (strcmp(cmd, "pos") == 0) {
        char *arg2 = strtok(NULL, " ");
        if (!arg2) { say("pos <id> <rad>\r\n> "); return; }
        float rad = strtof(arg2, NULL);
        if (rad < P_MIN || rad > P_MAX) { say("out of range\r\n> "); return; }
        motor_pos(id, rad);
        return;
    }

    say("?\r\n> ");
}


// call once after HAL_CAN_Start()
void init(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart)
{
    can = hcan; uart = huart;
    memset(fb, 0, sizeof(fb));
    HAL_CAN_ActivateNotification(can, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_UART_Receive_IT(uart, &rx_byte, 1);
    say("\r\nTOBY motor test -- enable / disable / zero / pos / stop / status\r\n> ");
}


// paste into MX_CAN1_Init() after HAL_CAN_Init(), accepts all frames
//
//   CAN_FilterTypeDef f = {0};
//   f.FilterBank = 0; f.FilterMode = CAN_FILTERMODE_IDMASK;
//   f.FilterScale = CAN_FILTERSCALE_32BIT;
//   f.FilterFIFOAssignment = CAN_RX_FIFO0; f.FilterActivation = ENABLE;
//   HAL_CAN_ConfigFilter(&hcan1, &f);
