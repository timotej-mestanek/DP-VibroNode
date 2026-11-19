/*
 * debug.cpp
 *
 *  Created on: Nov 19, 2026
 *      Author: timotej-mestanek
 */

#include "main.h"
#include <cstring>
#include <cstdio>
#include "debug.h"
#include "config.h"

extern UART_HandleTypeDef huart1;

static char uart_tx_buf[128];
static volatile uint8_t uart_tx_busy = 0;

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uart_tx_busy = 0;
    }
}

void debug_log(const char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
}

void debug_print_sample(const Sample &s, uint32_t idx) {
    if ((idx % PRINT_EVERY) != 0 || uart_tx_busy) {
        return;
    }

    int len = snprintf(uart_tx_buf, sizeof(uart_tx_buf),
                       "#%lu ACC:[%d,%d,%d] GYR:[%d,%d,%d]\r\n",
                       (unsigned long)idx,
                       s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
    if (len > 0) {
        uart_tx_busy = 1;
        if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)uart_tx_buf, len) != HAL_OK) {
            uart_tx_busy = 0;
        }
    }
}
