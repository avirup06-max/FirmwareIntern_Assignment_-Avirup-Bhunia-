/*
 * dma_uart_driver.c
 *
 *  Created on: Aug 18, 2025
 *      Author: avi
 */
#include "dma_uart.h"
#include "stm32g4xx.h"

#define APB1CLK_HZ   42000000UL
#define BAUDRATE     9600U

/* USART2 DMA mapping on STM32F4 (common): RX=DMA1 Stream5 Ch4, TX=DMA1 Stream6 Ch4 */
#define RX_STREAM    DMA1_Stream5
#define TX_STREAM    DMA1_Stream6
#define RX_TCIF      DMA_HISR_TCIF5
#define RX_TEIF      DMA_HISR_TEIF5
#define RX_DMEIF     DMA_HISR_DMEIF5
#define RX_FEIF      DMA_HISR_FEIF5
#define TX_TCIF      DMA_HISR_TCIF6
#define TX_TEIF      DMA_HISR_TEIF6
#define TX_DMEIF     DMA_HISR_DMEIF6
#define TX_FEIF      DMA_HISR_FEIF6

static volatile int s_err = 0;
static volatile int s_done = 0;

static void rcc_gpio_usart_dma_enable(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    __DSB(); __ISB();
}
static uint16_t br_oversample16(uint32_t pclk, uint32_t baud){
    uint32_t div = (pclk + (baud/2U)) / baud; return (uint16_t)div;
}
static void gpio_init_pa2_pa3_af7(void){
    GPIOA->MODER &= ~((3U<<(2*2)) | (3U<<(2*3)));
    GPIOA->MODER |=  ((2U<<(2*2)) | (2U<<(2*3)));
    GPIOA->OTYPER &= ~((1U<<2)|(1U<<3));
    GPIOA->OSPEEDR |= ((2U<<(2*2)) | (2U<<(2*3)));
    GPIOA->PUPDR &= ~((3U<<(2*2)) | (3U<<(2*3)));
    GPIOA->PUPDR |=  (0U<<(2*2)) | (1U<<(2*3));
    GPIOA->AFR[0] &= ~((0xF<<(4*2)) | (0xF<<(4*3)));
    GPIOA->AFR[0] |=  ((7U<<(4*2)) | (7U<<(4*3)));
}
static void clear_uart_errors(void){ volatile uint32_t t=USART2->SR; (void)t; t=USART2->DR; (void)t; }

int dma_uart_init(void){
    rcc_gpio_usart_dma_enable();
    gpio_init_pa2_pa3_af7();

    RCC->APB1RSTR |=  RCC_APB1RSTR_USART2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;

    USART2->CR1 = 0; USART2->CR2 = 0; USART2->CR3 = 0;
    USART2->CR1 &= ~(USART_CR1_OVER8 | USART_CR1_M | USART_CR1_PCE);
    USART2->CR2 &= ~USART_CR2_STOP;
    USART2->BRR = br_oversample16(APB1CLK_HZ, BAUDRATE);
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART2->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
    USART2->CR1 |= USART_CR1_UE;
    (void)USART2->SR; (void)USART2->DR;
    s_err=0; s_done=0;

    NVIC_SetPriority(DMA1_Stream5_IRQn,6);
    NVIC_SetPriority(DMA1_Stream6_IRQn,6);
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    return 0;
}

static void dma_rx_setup(uint8_t *rx, size_t len, int circular){
    RX_STREAM->CR &= ~DMA_SxCR_EN; while (RX_STREAM->CR & DMA_SxCR_EN) {}
    DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
    RX_STREAM->PAR  = (uint32_t)&USART2->DR;
    RX_STREAM->M0AR = (uint32_t)rx;
    RX_STREAM->NDTR = (uint16_t)len;
    RX_STREAM->CR = (4U<<DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_TCIE; /* per->mem */
    if (circular) RX_STREAM->CR |= DMA_SxCR_CIRC;
    RX_STREAM->FCR = 0;
}

static void dma_tx_setup(const uint8_t *tx, size_t len){
    TX_STREAM->CR &= ~DMA_SxCR_EN; while (TX_STREAM->CR & DMA_SxCR_EN) {}
    DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;
    TX_STREAM->PAR  = (uint32_t)&USART2->DR;
    TX_STREAM->M0AR = (uint32_t)tx;
    TX_STREAM->NDTR = (uint16_t)len;
    TX_STREAM->CR = (4U<<DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE; /* mem->per */
    TX_STREAM->FCR = 0;
}

int dma_uart_start(const uint8_t *tx, uint8_t *rx, size_t len, int rx_circular){
    s_err=0; s_done=0; clear_uart_errors();
    dma_rx_setup(rx, len, rx_circular);
    dma_tx_setup(tx, len);
    RX_STREAM->CR |= DMA_SxCR_EN;
    TX_STREAM->CR |= DMA_SxCR_EN;
    return 0;
}

int dma_uart_done(void){ return s_done; }
int dma_uart_had_error(void){ return s_err; }

void DMA1_Stream5_IRQHandler(void){
    uint32_t hisr = DMA1->HISR;
    if (hisr & RX_TCIF) { DMA1->HIFCR = DMA_HIFCR_CTCIF5; s_done = 1; }
    if (hisr & (RX_TEIF|RX_DMEIF|RX_FEIF)) { DMA1->HIFCR = DMA_HIFCR_CTEIF5|DMA_HIFCR_CDMEIF5|DMA_HIFCR_CFEIF5; s_err=1; }
}

void DMA1_Stream6_IRQHandler(void){
    uint32_t hisr = DMA1->HISR;
    if (hisr & TX_TCIF) { DMA1->HIFCR = DMA_HIFCR_CTCIF6; /* TX done */ }
    if (hisr & (TX_TEIF|TX_DMEIF|TX_FEIF)) { DMA1->HIFCR = DMA_HIFCR_CTEIF6|DMA_HIFCR_CDMEIF6|DMA_HIFCR_CFEIF6; s_err=1; }
}
