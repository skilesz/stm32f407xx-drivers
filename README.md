# stm32f407xx-drivers    

This project is my attempt at writing drivers for different peripherals connected to the STM32F407xx family of microcontrollers. To use any and all drivers, put all files from the `inc` folder in your include directory and all files from the `src` folder in your source directory. Then, just use `#include "stm32f407.h"` at the top of your `main.c` file.

## Current Drivers
Click on a name to see the relevant peripheral driver documentation.

- [GPIO](/GPIO.md)

## Upcoming Drivers
These are drivers I am planning to implement in the near future. Feel free to request a particular peripheral!

- SPI
- I2C
- UART
- RTC
- TIM
