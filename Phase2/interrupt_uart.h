/*
 * interrupt_uart_driver.h
 *
 *  Created on: Aug 18, 2025
 *      Author: avi
 */
 
#ifndef INC_UART_DRIVER_H_
#define INC_UART_DRIVER_H_

#include "stm32g4xx.h"
#include <stdint.h>

#define UART_TX_SIZE 50
#define UART_RX_SIZE 50

/* --- Polling API (Part A) --- */
void UART_Init(void);
void UART_Transmit(uint8_t *data, uint32_t len);
void UART_Receive(uint8_t *buffer, uint32_t len);

/* --- Interrupt API (Part B) --- */
/* Provide pointers to buffers (owner = caller, e.g. main.c) */
void UART_Transmit_IT(uint8_t *data, uint16_t len);
void UART_Receive_IT(uint8_t *buffer, uint16_t len);

/* Status flags (readable by main/debugger) */
extern volatile uint8_t uart_tx_done;
extern volatile uint8_t uart_rx_done;

/* Error counters (readable for debug) */
extern volatile uint32_t uart_err_ore;
extern volatile uint32_t uart_err_fe;
extern volatile uint32_t uart_err_ne;

#endif /* INC_UART_DRIVER_H_ */
