/*
 * dma_uart_driver.c
 *
 *  Created on: Aug 18, 2025
 *      Author: avi
 */
 
#ifndef DMA_UART_H
#define DMA_UART_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

int  dma_uart_init(void);
int  dma_uart_start(const uint8_t *tx, uint8_t *rx, size_t len, int rx_circular);
int  dma_uart_done(void);
int  dma_uart_had_error(void);

/* Hook to vector table */
void DMA1_Stream5_IRQHandler(void); /* RX */
void DMA1_Stream6_IRQHandler(void); /* TX */

#ifdef __cplusplus
}
#endif
#endif /* DMA_UART_H */
