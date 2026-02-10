Top-level structure:
- Core/: STM32Cube-generated init code (main.c, usart.c, dma.c, stm32f4xx_it.c, etc.).
- Drivers/: STM32 HAL + CMSIS device/driver sources.
- Middlewares/: FreeRTOS sources.
- Bsp/: board support drivers (bsp_uart.c, bsp_can.c, bsp_dwt.c, bsp_pwr.c) and protocol parsing (UartProtocol.c).
- User/: application logic and tasks (UserTask.c, motor control, UartModule.c).
- MDK-ARM/: Keil project and build artifacts (Dart_Ctrl.uvprojx).
- Docs/: design and usage docs (notably BSP_UART usage).