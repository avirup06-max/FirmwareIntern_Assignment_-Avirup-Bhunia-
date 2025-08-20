/*
 * polling_uart_driver.h
 *
 *  Created on: Aug 18, 2025
 *      Author: avi
 */
 
#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "stm32g4xx.h"   // Device header for STM32G431
#include <stdint.h>

// UART buffer sizes
#define UART_TX_SIZE  50
#define UART_RX_SIZE  50

// Function prototypes
void UART_Init(void);
void UART_Transmit(uint8_t *data, uint32_t len);
void UART_Receive(uint8_t *buffer, uint32_t len);

#endif

