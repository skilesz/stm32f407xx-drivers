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

For example, to enable pin PD12 as an input pin with the internal pull-up resistor enabled, you would write something like the following:

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

The following functions are made available to the user to use the GPIO peripheral. The handle variable used in the *EXAMPLE USAGE* lines is the `input` handle created above.

<br/>

### int8_t GPIO_Clock(GPIO_RegDef_t* pGPIOx, uint8_t enable)
- **DESCRIPTION**: Enables or disables peripheral clock for given GPIO port
- **PARAMETERS**:
  - *pGPIOx*: one of 11 GPIO ports (GPIOA, GPIOB, ..., GPIOK)
  - *enable*: one of 2 macros (ENABLE or DISABLE)
- **RETURN**:
  - 0 = success
  - NEGATIVE = see [error codes](#error-codes)
- **EXAMPLE USAGE**: `GPIO_Clock(GPIOD, ENABLE);`

<br/>

### int8_t GPIO_InitPin(GPIO_Handle_t* pGPIOHandle)
- **DESCRIPTION**: Initializes pin with specified properties
- **PARAMETERS**:
  - *pGPIOHandle*: handle of pin to be configured
- **RETURN**:
  - 0 = success
  - NEGATIVE = see [error codes](#error-codes)
- **EXAMPLE USAGE**: `GPIO_InitPin(input)`

<br/>

### int8_t GPIO_ResetPort(GPIO_RegDef_t* pGPIOx)
- **DESCRIPTION**: Resets all pins of specified port
- **PARAMETERS**:
  - *pGPIOx*: one of 11 GPIO ports (GPIOA, GPIOB, ..., GPIOK)
- **RETURN**:
  - 0 = success
  - NEGATIVE = see [error codes](#error-codes)
- **EXAMPLE USAGE**: `GPIO_ResetPort(GPIOD)`

## Error Codes
