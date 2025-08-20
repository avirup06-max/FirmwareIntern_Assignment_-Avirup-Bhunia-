/*
 * interrupt_uart_driver.c
 *
 *  Created on: Aug 18, 2025
 *      Author: avi
 */
 
#include "uart_driver.h"
#include <string.h> // optional if you use memcmp in main

/* ---- Config ---- */
#define UART_PCLK      16000000UL   // PCLK for USART2 (HSI16 default in your SystemClock_Config)
#define UART_BAUD      9600U

/* ---- Status / error variables visible to main/debugger ---- */
volatile uint8_t  uart_tx_done = 0;
volatile uint8_t  uart_rx_done = 0;
volatile uint32_t uart_err_ore = 0;
volatile uint32_t uart_err_fe  = 0;
volatile uint32_t uart_err_ne  = 0;

/* ---- Internal pointers/state used by interrupt mode ---- */
static volatile uint8_t *g_tx_buf = 0;
static volatile uint8_t *g_rx_buf = 0;
static volatile uint16_t g_tx_len = 0;
static volatile uint16_t g_rx_len = 0;
static volatile uint16_t g_tx_idx = 0;
static volatile uint16_t g_rx_idx = 0;

/* ----------------- Helper: configure GPIOA PA2/PA3 as AF7 (USART2) ----------------- */
static void uart_gpio_init_pa2_pa3(void)
{
    /* Enable GPIOA clock */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    /* PA2, PA3 -> Alternate function */
    GPIOA->MODER &= ~((3U << (2*2)) | (3U << (3*2)));
    GPIOA->MODER |=  ((2U << (2*2)) | (2U << (3*2)));

    /* AFRL: AF7 (USART2) for PA2, PA3 */
    GPIOA->AFR[0] &= ~((0xFu << (4*2)) | (0xFu << (4*3)));
    GPIOA->AFR[0] |=  ((7U << (4*2)) | (7U << (4*3)));

    /* Pull-up RX (PA3), TX no pull */
    GPIOA->PUPDR &= ~((3U << (2*2)) | (3U << (3*2)));
    GPIOA->PUPDR |=  ((0U << (2*2)) | (1U << (3*2)));

    /* High speed */
    GPIOA->OSPEEDR &= ~((3U << (2*2)) | (3U << (3*2)));
    GPIOA->OSPEEDR |=  ((2U << (2*2)) | (2U << (3*2)));
}

/* ----------------- Initialize USART2 (common for polling & IRQ) ----------------- */
void UART_Init(void)
{
    /* enable clocks */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;      /* already done in gpio init but safe */
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

    uart_gpio_init_pa2_pa3();

    /* Reset USART2 peripheral and clear config */
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_USART2RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_USART2RST;

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    /* Baud: use simple rounding for OVER16 */
    uint32_t brr = (UART_PCLK + (UART_BAUD/2U)) / UART_BAUD;
    USART2->BRR = brr;

    /* 8-bit, no parity, 1 stop are defaults if M and PCE cleared; enable TE/RE */
    USART2->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1 | USART_CR1_PCE);
    USART2->CR2 &= ~USART_CR2_STOP;
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    /* Enable USART */
    USART2->CR1 |= USART_CR1_UE;

    /* Small delay to let UE take effect */
    for (volatile int i=0;i<200;i++) __NOP();

    /* Enable NVIC for USART2 (priority 5, adjust if needed) */
    NVIC_SetPriority(USART2_IRQn, 5);
    NVIC_EnableIRQ(USART2_IRQn);
}

/* ----------------- Polling API (Part A) - kept for compatibility ----------------- */
void UART_Transmit(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; ++i) {
        while (!(USART2->ISR & USART_ISR_TXE)) {
            /* optional: check and clear errors while waiting */
            uint32_t isr = USART2->ISR;
            if (isr & USART_ISR_ORE) { uart_err_ore++; USART2->ICR = USART_ICR_ORECF; (void)USART2->RDR; }
            if (isr & USART_ISR_FE)  { uart_err_fe++;  USART2->ICR = USART_ICR_FECF; }
            if (isr & USART_ISR_NE)  { uart_err_ne++;  USART2->ICR = USART_ICR_NECF; }
        }
        USART2->TDR = data[i];
    }
    /* Wait until Transmission Complete (TC) */
    while (!(USART2->ISR & USART_ISR_TC)) { /* wait */ }
}

void UART_Receive(uint8_t *buffer, uint32_t len)
{
    for (uint32_t i = 0; i < len; ++i) {
        while (!(USART2->ISR & USART_ISR_RXNE)) {
            uint32_t isr = USART2->ISR;
            if (isr & USART_ISR_ORE) { uart_err_ore++; USART2->ICR = USART_ICR_ORECF; (void)USART2->RDR; }
            if (isr & USART_ISR_FE)  { uart_err_fe++;  USART2->ICR = USART_ICR_FECF; }
            if (isr & USART_ISR_NE)  { uart_err_ne++;  USART2->ICR = USART_ICR_NECF; }
        }
        buffer[i] = (uint8_t)USART2->RDR;
    }
}

/* ----------------- Interrupt API (Part B) ----------------- */

/* Start an interrupt-driven transmit. Caller owns the buffer memory. */
void UART_Transmit_IT(uint8_t *data, uint16_t len)
{
    /* set pointer/state */
    g_tx_buf = (volatile uint8_t*)data;
    g_tx_len = len;
    g_tx_idx = 0;
    uart_tx_done = 0;

    /* If TXE is set now, write first byte to start transfer; otherwise ISR will handle */
    if (USART2->ISR & USART_ISR_TXE) {
        if (g_tx_idx < g_tx_len) {
            USART2->TDR = g_tx_buf[g_tx_idx++];
        }
    }
    /* Enable TXE interrupt to continue sending remaining bytes */
    USART2->CR1 |= USART_CR1_TXEIE;
}

/* Start interrupt-driven receive. Caller provides buffer. */
void UART_Receive_IT(uint8_t *buffer, uint16_t len)
{
    g_rx_buf = (volatile uint8_t*)buffer;
    g_rx_len = len;
    g_rx_idx = 0;
    uart_rx_done = 0;

    /* Enable RXNE interrupt (ISR will fill buffer) */
    USART2->CR1 |= USART_CR1_RXNEIE;
}

/* ----------------- USART2 IRQ Handler (overrides weak handler) ----------------- */
void USART2_IRQHandler(void)
{
    uint32_t isr = USART2->ISR;

    /* --- Errors first --- */
    if (isr & USART_ISR_ORE) {
        uart_err_ore++;
        USART2->ICR = USART_ICR_ORECF;
        (void)USART2->RDR; /* flush */
    }
    if (isr & USART_ISR_FE) {
        uart_err_fe++;
        USART2->ICR = USART_ICR_FECF;
    }
    if (isr & USART_ISR_NE) {
        uart_err_ne++;
        USART2->ICR = USART_ICR_NECF;
    }

    /* --- RXNE (data received) --- */
    if ((isr & USART_ISR_RXNE) && (USART2->CR1 & USART_CR1_RXNEIE)) {
        if (g_rx_idx < g_rx_len) {
            g_rx_buf[g_rx_idx++] = (uint8_t)USART2->RDR;
            if (g_rx_idx >= g_rx_len) {
                /* done receiving */
                USART2->CR1 &= ~USART_CR1_RXNEIE;
                uart_rx_done = 1;
            }
        } else {
            /* buffer full -- discard */
            (void)USART2->RDR;
        }
    }

    /* --- TXE (transmit data register empty) --- */
    if ((isr & USART_ISR_TXE) && (USART2->CR1 & USART_CR1_TXEIE)) {
        if (g_tx_idx < g_tx_len) {
            USART2->TDR = g_tx_buf[g_tx_idx++];
            /* If last byte was just written, TXE will be cleared and then set once shifting occurs.
               We keep tx_done when last byte has been queued; if you need TC (physical complete), check TC flag. */
            if (g_tx_idx >= g_tx_len) {
                /* stop generating TXE interrupts once all bytes queued */
                USART2->CR1 &= ~USART_CR1_TXEIE;
                uart_tx_done = 1;
            }
        } else {
            USART2->CR1 &= ~USART_CR1_TXEIE;
            uart_tx_done = 1;
        }
    }
}

