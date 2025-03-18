# GPIO Driver

This page details the use of the GPIO driver, including the API, structures, and error codes. The STM32F407XX family of microcontrollers contain up to 11 ports (GPIOA - GPIOK), each with up to 16 individual pins.

## Getting Started

To begin working with a specific GPIO pin, you must first create a GPIO handle struct, which is defined as follows:

```c
typedef struct {
	GPIO_RegDef_t* pGPIOx;					// Pointer to GPIO port and registers
	uint8_t pinNumber;					// Pin number (0 - 15)
	GPIO_PinConfig_t config;				// GPIO pin config settings
} GPIO_Handle_t;
``` 


- `pGPIOx`: one of 11 GPIO ports (GPIOA, GPIOB, ..., GPIOK)
- `pinNumber`: any number from 0 - 15
- `config`: pin configuration struct

<br/>

The pin configuration struct is defined as follows:

```c
typedef struct {
	uint8_t mode;					// Pin mode (@GPIO_MODES)
	uint8_t outputType;				// Pin output type (@GPIO_OUTPUT_TYPES)
	uint8_t speed;					// Pin speed (@GPIO_SPEEDS)
	uint8_t pupd;					// Pin pull-up or pull-down configuration (@GPIO_PUPD)
	uint8_t altFun;					// Pin alternate function number (0 - 15)
} GPIO_PinConfig_t;
```

- `mode`: one of the following GPIO mode macros
  - `GPIO_MODE_IN`: input
  - `GPIO_MODE_OUT`: output
  - `GPIO_MODE_ALT`: alternate function
  - `GPIO_MODE_ANALOG`: analog
  - `GPIO_MODE_IN_RE`: input with interrupt triggered on rising edge
  - `GPIO_MODE_IN_FE`: input with interrupt triggered on falling edge
  - `GPIO_MODE_IN_RFE`: input with interrupt triggered on rising and falling edges
- `outputType`: one of the following GPIO output type macros
  - `GPIO_OT_PP`: push-pull
  - `GPIO_OT_OD`: open-drain
- `speed`: one of the following GPIO output speed macros
  - `GPIO_SPEED_LOW`: low speed
  - `GPIO_SPEED_MED`: medium speed
  - `GPIO_SPEED_HI`: high speed
  - `GPIO_SPEED_VHI`: very high speed
- `pupd`: one of the following GPIO pull-up/pull-down configuration macros
  - `GPIO_PUPD_NONE`: no pull-up/pull-down
  - `GPIO_PUPD_PU`: pull-up resistor enabled
  - `GPIO_PUPD_PD`: pull-down resistor enabled
- `altFun`: any number from 0 - 15

<br/>

For example, to configure pin PD12 as an input pin with the internal pull-up resistor enabled, you would write something like the following:

```c
GPIO_Handle_t input;

input.pGPIOx = GPIOD;
input.pinNumber = 12;
input.config.mode = GPIO_MODE_IN;
input.config.pupd = GPIO_PUPD_PU;
```

<br/>

After creating a GPIO handle, you can use it in the functions defined in the API.

## API

The following functions are made available to the user to utilize the GPIO peripheral. All functions will report a specific error by returning a negative value as specified in the [Error Codes](#error-codes) section.

<br/>

### GPIO_Clock()
```c
/*
 * @fn				GPIO_Clock
 *
 * @desc			Enables or disables peripheral clock for given GPIO port
 *
 * @param			pGPIOx: base address of GPIO port
 * @param			enable: ENABLE or DISABLE macros
 *
 * @return			0		-> success
 *				NEGATIVE	-> see @ERROR_CODES
 */

int8_t GPIO_Clock(GPIO_RegDef_t* pGPIOx, uint8_t enable)
```
**EXAMPLE USAGE**  
*Enable clock of GPIO port D*
```c
GPIO_Clock(GPIOD, ENABLE);
````

<br/>

### GPIO_InitPin()
```c
/*
 * @fn				GPIO_InitPin
 *
 * @desc			Initializes pin with specified properties
 *
 * @param			pGPIOHandle: settings of pin to be configured
 *
 * @return			0		-> success
 * 				NEGATIVE	-> see @ERROR_CODES
 */

int8_t GPIO_InitPin(GPIO_Handle_t* pGPIOHandle)
```
**EXAMPLE USAGE**  
*Initialize pin PD12 (as specified by `input` handle created in [Getting Started](#getting-started))*
```c
GPIO_InitPin(&input);
```

<br/>

### GPIO_ResetPort()
```c
/*
 * @fn				GPIO_ResetPort
 *
 * @desc			Resets specified port
 *
 * @param			pGPIOx: base address of GPIO port
 *
 * @return			0		-> success
 * 				NEGATIVE	-> see @ERROR_CODES
 */

int8_t GPIO_ResetPort(GPIO_RegDef_t* pGPIOx)
```
**EXAMPLE USAGE**  
*Reset all pins of GPIO port D*
```c
GPIO_ResetPort(GPIOD);
```

<br/>

### GPIO_ReadPin()
```c
/*
 * @fn				GPIO_ReadPin
 *
 * @desc			Reads specified pin
 *
 * @param			pGPIOHandle: handle of pin to be read from
 *
 * @return			0x0 - 0x1	-> success
 * 				NEGATIVE	-> see @ERROR_CODES
 */

int8_t GPIO_ReadPin(GPIO_Handle_t* pGPIOHandle)
```
**EXAMPLE USAGE**  
*Read value of pin PD12 (referenced using `input` handle created in [Getting Started](#getting-started))*
```c
GPIO_ReadPin(&input);
```

<br/>

### GPIO_ReadPort()
```c
/*
 * @fn				GPIO_ReadPort
 *
 * @desc			Reads all pins of specified port
 *
 * @param			pGPIOx: base address of GPIO port
 *
 * @return			0x0 - 0xFFFF	-> success
 * 				NEGATIVE	-> see @ERROR_CODES
 */

int32_t GPIO_ReadPort(GPIO_RegDef_t* pGPIOx)
```
**EXAMPLE USAGE**  
*Read value of all pins of GPIO port D*
```c
GPIO_ReadPort(GPIOD);
```

<br/>

### GPIO_WritePin()
```c
/*
 * @fn				GPIO_WritePin
 *
 * @desc			Writes value to specified pin
 *
 * @param			pGPIOHandle: handle of pin to be written to
 * @param 			set: SET or RESET macros
 *
 * @return			0		-> success
 * 				NEGATIVE	-> see @ERROR_CODES
 */

int8_t GPIO_WritePin(GPIO_Handle_t* pGPIOHandle, uint8_t set)
```
**EXAMPLE USAGE**  
*Set output value of pin PD12 to 1 (referenced using `input` handle created in [Getting Started](#getting-started))*
```c
GPIO_WritePin(&input, SET);
```

### GPIO_WritePort()
```c
/*
 * @fn				GPIO_WritePort
 *
 * @desc			Writes value to specified port
 *
 * @param			pGPIOx: base address of GPIO port
 * @param			val: value to be written to each pin
 *
 * @return			0		-> success
 * 				NEGATIVE	-> see @ERROR_CODES
 */

int8_t GPIO_WritePort(GPIO_RegDef_t* pGPIOx, uint16_t val)
```
**EXAMPLE USAGE**  
*Set all pins of GPIO port D to 1*
```c
GPIO_WritePort(GPIOD, 0xFFFF);
```

### GPIO_TogglePin()
```c
/*
 * @fn				GPIO_TogglePin
 *
 * @desc			Toggle specified pin
 *
 * @param			pGPIOHandle: handle of the pin to be toggled
 *
 * @return			0		-> success
 * 				NEGATIVE	-> see @ERROR_CODES
 */

int8_t GPIO_TogglePin(GPIO_Handle_t* pGPIOHandle)
```
**EXAMPLE USAGE**  
*Toggle pin PD12 (referenced using `input` handle created in [Getting Started](#getting-started))*
```c
GPIO_TogglePin(&input);
```

### GPIO_IRQEnable()
```c
/*
 * @fn				GPIO_IRQEnable
 *
 * @desc			Enables specified IRQ
 *
 * @param			IRQNumber: number of IRQ (see IRQNumber Macros)
 * @param			enable: ENABLE or DISABLE macros
 *
 * @return			0		-> success
 * 				NEGATIVE	-> see @ERROR_CODES
 */

int8_t GPIO_IRQEnable(uint8_t IRQNumber, uint8_t enable)
```
**`IRQNumber` Macros**
- `IRQ_EXTI0`
- `IRQ_EXTI1`
- `IRQ_EXTI2`
- `IRQ_EXTI3`
- `IRQ_EXTI4`
- `IRQ_EXTI9_5`
- `IRQ_EXTI15_10`
  
**EXAMPLE USAGE**  
*Enable IRQ handler to service pin PD12 (corresponds to EXTI15_10 handler)*
```c
GPIO_IRQEnable(IRQ_EXTI15_10, ENABLE);
```

### GPIO_IRQPriority()
```c
/*
 * @fn 				GPIO_IRQPriority
 *
 * @desc 			Sets the priority for given IRQ number
 *
 * @param			IRQNumber: number of IRQ (see IRQNumber Macros)
 * @param			priority: priority to be set (0 - 15)
 *
 * @return			0		-> success
 * 				NEGATIVE	-> see @ERROR_CODES
 */

int8_t GPIO_IRQPriority(uint8_t IRQNumber, uint8_t priority)
```
**`IRQNumber` Macros**
- `IRQ_EXTI0`
- `IRQ_EXTI1`
- `IRQ_EXTI2`
- `IRQ_EXTI3`
- `IRQ_EXTI4`
- `IRQ_EXTI9_5`
- `IRQ_EXTI15_10`
  
**EXAMPLE USAGE**  
*Set EXTI15_10 IRQ handler with a priority of 1*
```c
GPIO_IRQPriority(IRQ_EXTI15_10, 1);
```

### GPIO_IRQClearPending()
```c
/*
 * @fn				GPIO_IRQClearPending
 *
 * @desc			Clears interrupt pending bit for specified pin
 *
 * @param			pinNumber: number of pin that triggered interrupt
 * @param
 *
 * @return			0		-> success
 * 				NEGATIVE	-> see @ERROR_CODES
 */

int8_t GPIO_IRQClearPending(uint8_t pinNumber)
```
**EXAMPLE USAGE**  
*Clear pending bit for interrupt triggered by pin PD12*
```c
GPIO_IRQClearPending(12);
```

## Error Codes

| Code | Meaning |
| ---- | ------- |
| -1   | null port pointer 				|
| -2   | invalid port pointer 				|
| -3   | null handle pointer				|
| -4   | invalid pin number 				|
| -5   | invalid mode 					|
| -6   | invalid output type 				|
| -7   | invalid speed 					|
| -8   | invalid pull-up/pull-down configuration 	|
| -9   | invalid alternate function			|
| -10  | invalid enable value				|
| -11  | specified pin does not support interrupt mode	|
| -12  | invalid set value				|
| -13  | invalid IRQ Number				|
| -14  | invalid IRQ priority				|


## Full Example
The following is a full example of an application that uses this GPIO driver to toggle an LED connected to pin PA3 when a button connected to pin PB8 is pressed (button is active low, meaning we trigger an interrupt on the falling edge to handle the button press).
```c
#include "stm32f407xx.h"


// Declare global handles for led and button pins
GPIO_Handle_t led;
GPIO_Handle_t button;


int main(void) {

	// Define configuration for pin PA3 (led)
	led.pGPIOx = GPIOA;
	led.pinNumber = 3;
	led.config.mode = GPIO_MODE_OUT;
	led.config.outputType = GPIO_OT_PP;
	led.config.speed = GPIO_SPEED_LOW;
	led.config.pupd = GPIO_PUPD_NONE;

	// Define configuration for pin PB8 (button)
	button.pGPIOx = GPIOB;
	button.pinNumber = 8;
	button.config.mode = GPIO_MODE_IN_FE;
	button.config.pupd = GPIO_PUPD_PU;

	// Enable relevant port clocks
	GPIO_Clock(GPIOA, ENABLE);
	GPIO_Clock(GPIOB, ENABLE);

	// Initialize pins with specified configurations
	GPIO_InitPin(&led);
	GPIO_InitPin(&button);

	// Enable corresponding IRQ handler for pin PB8 (EXTI9_5)
	GPIO_IRQEnable(IRQ_EXTI9_5, ENABLE);

	// Set priority of EXTI9_5 IRQ handler to 5
	GPIO_IRQPriority(IRQ_EXTI9_5, 5);

	// Do nothing
	while(1);
}


// IRQ handler to service interrupts on pin PB8
void EXTI9_5_IRQHandler(void) {

	// Clear interrupt pending bit
	GPIO_IRQClearPending(8);

	// Toggle pin PA3 (led)
	GPIO_TogglePin(&led);
}
```
