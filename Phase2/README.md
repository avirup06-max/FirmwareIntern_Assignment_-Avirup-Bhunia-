STM32G431 USART Driver Project
Overview
This project implements a custom UART/USART driver for the STM32G431 microcontroller in three incremental stages:
1.	Part A – Polling Mode Communication
2.	Part B – Interrupt-Based Loopback
3.	Part C – DMA-Based Transmission/Reception
All development and testing were performed in STM32CubeIDE, with build verification and debugging done using the integrated tools.
________________________________________
Project Structure
Core/
├── Inc/
│   ├── uart_driver.h   # Driver header (function prototypes, constants)
│   └── dma_driver.h    # Added for Part C
├── Src/
│   ├── main.c          # Application code
│   ├── uart_driver.c   # UART driver implementation
│   └── dma_driver.c    # DMA driver implementation (Part C)
________________________________________
Part A – Polling
•	Implemented basic UART transmit/receive using blocking (polling) methods.
•	Verified that Build Finished output confirmed successful compilation.
________________________________________
Part B – Interrupt and Loopback
•	Configured UART with interrupts for non-blocking communication.
•	Implemented a loopback test: transmitted data was internally received and verified in the rx_data buffer.
•	Debug session confirmed rx_data matched tx_data.
________________________________________
Part C – DMA Integration
•	Added DMA-based UART communication.
•	Circular mode enabled continuous data reception.
•	DMA NDTR register monitored for buffer status.
•	Improved efficiency by reducing CPU intervention during high-throughput transfers.
________________________________________
Technical Notes
GPIO Speed and Signal Integrity
•	Higher GPIO speed improves rise/fall times but increases EMI.
•	Medium or high-speed GPIO output is typically selected for UART pins to balance integrity and noise.
Baud Rate Calculation (Oversampling by 8/16)
•	Oversampling by 16: higher tolerance to clock mismatch, lower noise sensitivity.
•	Oversampling by 8: allows higher baud rates but requires tighter clock precision.
•	BRR register is computed differently depending on oversampling mode.
DMA NDTR and Circular Mode
•	NDTR (Number of Data to Transfer) decrements with each byte moved.
•	In circular mode, NDTR automatically reloads after reaching zero, ensuring continuous reception without reinitialization.
USART Error Flags
•	ORE (Overrun Error): Data lost because RX register was not read in time.
•	FE (Framing Error): Stop bit not detected properly.
•	NE (Noise Error): Line noise affected reception.
•	Errors must be cleared by reading the status register and data register.
Peripheral Reset and Flag Clearing
•	Peripheral reset performed via RCC reset register.
•	Flags cleared either by register read sequences or explicit flag reset bits in USART registers.
Debugging Techniques
•	Used CubeIDE Debug Perspective to observe variables (tx_data, rx_data).
•	Verified interrupt firing and DMA transfers by monitoring memory buffers.
•	Breakpoints in ISR handlers confirmed proper execution flow.
________________________________________
How to Run
1.	Clone repository into STM32CubeIDE workspace.
2.	Build project (hammer icon).
3.	Run simulation/debug (with QEMU or supported board).
4.	For loopback test (Part B), verify rx_data matches tx_data.
5.	For DMA (Part C), monitor buffer updates in circular mode.
________________________________________
Author Avirup Bhunia
