# GPIO Driver

This page details the use of the GPIO driver, including the API, structures, and error codes. The STM32F407XX family of microcontrollers contain 11 ports (GPIOA - GPIOK), each with up to 16 individual pins.

## Getting Started

To begin working with a specific GPIO pin, you must first create a GPIO handle struct, which is defined as follows:

```
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

```
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

```
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
```
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
```
GPIO_Clock(GPIOD, ENABLE);
````

<br/>

### GPIO_InitPin()
```
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
```
GPIO_InitPin(input);
```

<br/>

### GPIO_ResetPort()
```
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
```
GPIO_ResetPort(GPIOD);
```

<br/>

### GPIO_ReadPin()
```
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
```
GPIO_ReadPin(input);
```

<br/>

### GPIO_ReadPort()
```
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
```
GPIO_ReadPort(GPIOD);
```

<br/>

### GPIO_WritePin()
```
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
```
GPIO_WritePin(input, SET);
```


## Error Codes
