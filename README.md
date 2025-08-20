# STM32F411 Development Board Assignment

## Overview
This repository contains all deliverables for the STM32 assignment, covering Phase 1 through Phase 3.  
The work demonstrates register-level understanding, software driver development (polling, interrupt, DMA), and practical board bring-up methodology.

---

# Phase 1 – Learning Summary
### Key Concepts Learned
- GPIO configuration and alternate function mapping  
- RCC clock tree and peripheral clock enabling  
- USART/UART communication basics  
- DMA controller (normal and circular transfer modes)  
- TIMER peripherals for delays, PWM generation, and input capture  
- I2C and SPI configuration and protocol usage  
- NVIC interrupt structure and vector table understanding  

### Peripheral-Specific Challenges and Takeaways
- Mapping alternate functions correctly was critical for UART/I2C/SPI GPIO configuration.  
- Understanding the clock tree required attention to APB1/APB2 prescalers and PLL configuration.  
- For USART, framing errors and overrun conditions needed explicit handling in firmware.  
- DMA transfers required careful configuration of channel direction, increment mode, and circular buffering.  
- Timer capture/compare demanded precise prescaler and ARR selection for stable PWM and input capture.  
- I2C implementation highlighted the importance of acknowledging signals and bus recovery after errors.  
- NVIC configuration clarified interrupt priority grouping and preemption rules.  

---

# Phase 2 – Software UART Loopback Drivers
### Part A – Polling Driver
- Implemented uart_polling_* functions for initialization, transmit, and receive.  
- Payload: 50-byte fixed string (A–Z, digits, symbols, CRLF).  
- Error handling included overrun, framing, and noise errors with soft-reset recovery.  
- Verification via polling readback of transmitted bytes.  

### Part B – Interrupt Driver
- NVIC-based ISR simulation with USART2_IRQHandler.  
- RXNE and TXE interrupts drive data transfer without blocking.  
- ISR clears error flags (ORE, FE, NE) by simulated SR/DR access.  
- Completion detected via received byte counter, ensuring all 50 bytes looped back.  

### Part C – DMA Driver
- DMA configured for memory-to-peripheral (TX) and peripheral-to-memory (RX).  
- NDTR decrements until transfer complete; interrupt flag marks completion.  
- Circular mode supported for continuous transfers.  
- Demonstrated non-blocking, low-CPU overhead UART communication.  

---

# Phase 3 – Hardware Bring-Up
### Schematic and PCB
- Custom STM32F411 board designed with AP2112-3.3 regulator, 8 MHz HSE crystal, USB FS interface, and SWD debug header.  
- Decoupling capacitors placed near every VDD pin; VCAP1 capacitor placed correctly.  
- USB lines include 22 Ω resistors and ESD protection.  

### Bring-Up Procedure
- Pre-power checks for shorts on 3V3 and VBUS lines.  
- Power-up verification of voltage rails (5 V, 3.3 V) and NRST line.  
- SWD connection validated with ST-Link and STM32CubeProgrammer.  
- Firmware sanity check: PC13 LED toggle, PA0 button input, USART2 loopback at 9600 baud.  
- USB CDC enumeration tested with VBUS sense divider on PA9.  

### Troubleshooting and Recovery
- Added test points for VDD, GND, NRST, UART RX/TX.  
- Provided recovery for UART error flags, crystal startup issues, and USB enumeration failures.  

---

# How to Build and Run
### Simulation (No Hardware Required)
- Polling, interrupt, and DMA loopback drivers include *standalone C simulator code*.  
- Compile with GCC or Clang:  
  ```bash
  gcc -std=c11 -O2 -o sim phase2_polling.c
  ./sim

Output shows PASS or FAIL along with received payload.


Hardware Bring-Up

Load loopback firmware via SWD programmer.

Connect USB to PC and test virtual COM port communication.

Observe LED toggle and verify UART/USB loopback data.



---

Conclusion

This assignment demonstrates a complete learning and implementation cycle:

Phase 1: Gaining theoretical and peripheral-level knowledge.

Phase 2: Developing low-level drivers for UART (polling, interrupt, DMA).

Phase 3: Translating understanding into practical hardware design and bring-up.


The project validates both conceptual understanding and the ability to apply it in real-world embedded systems development.
