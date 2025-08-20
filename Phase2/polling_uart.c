/*
 * polling_uart_driver.c
 *
 *  Created on: Aug 18, 2025
 *      Author: avi
 */

#include "uart_driver.h"

// UART2 base address (STM32G431: usually PA2 = TX, PA3 = RX)
#define USART2_BASE (0x40004400UL)
#define USART2      ((USART_TypeDef *) USART2_BASE)

void UART_Init(void)
{
    // Enable clock for GPIOA and USART2
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;   // Enable GPIOA clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; // Enable USART2 clock

    // Configure PA2 as AF7 (USART2_TX), PA3 as AF7 (USART2_RX)
    GPIOA->MODER &= ~((3UL<<(2*2)) | (3UL<<(3*2))); // clear mode
    GPIOA->MODER |=  ((2UL<<(2*2)) | (2UL<<(3*2))); // alternate function
    GPIOA->AFR[0] |= (7UL<<(2*4)) | (7UL<<(3*4));   // AF7 for USART2
    GPIOA->OSPEEDR |= (3UL<<(2*2)) | (3UL<<(3*2));  // High speed

    // Configure USART2
    USART2->BRR = 80000000 / 9600; // Assuming APB1 clock = 80 MHz, baud=9600
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable TX, RX, USART
}

void UART_Transmit(uint8_t *data, uint32_t len)
{
    for(uint32_t i = 0; i < len; i++)
    {
        while(!(USART2->ISR & USART_ISR_TXE_TXFNF)); // Wait until TX buffer empty
        USART2->TDR = data[i]; // Send byte
    }
}

void UART_Receive(uint8_t *buffer, uint32_t len)
{
    for(uint32_t i = 0; i < len; i++)
    {
        while(!(USART2->ISR & USART_ISR_RXNE_RXFNE)); // Wait until data received
        buffer[i] = (uint8_t)(USART2->RDR & 0xFF);    // Read received byte
    }
}
