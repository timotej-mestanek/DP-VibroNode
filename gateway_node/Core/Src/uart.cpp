/*
 * debug.cpp
 *
 *  Created on: Nov 19, 2026
 *      Author: timotej-mestanek
 */


#include <uart.h>
#include "main.h"
#include <cstring>
#include <cstdio>

extern UART_HandleTypeDef huart1;
static volatile uint8_t uart_tx_busy = 0;


extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uart_tx_busy = 0;
    }
}


void debug_log(const char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
}



