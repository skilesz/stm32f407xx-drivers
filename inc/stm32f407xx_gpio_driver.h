/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Feb 28, 2025
 *      Author: Zach Skiles (skilesz)
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"




/**********************Structures**********************/

/*
 * Config structure
 */

typedef struct {
	uint8_t mode;					// Pin mode (@GPIO_MODES)
	uint8_t outputType;				// Pin output type (@GPIO_OUTPUT_TYPES)
	uint8_t speed;					// Pin speed (@GPIO_SPEEDS)
	uint8_t pupd;					// Pin pull-up or pull-down configuration (@GPIO_PUPD)
	uint8_t altFun;					// Pin alternate function number (0 - 15)
} GPIO_PinConfig_t;

/*
 * Handle structure
 */

typedef struct {
	GPIO_RegDef_t* pGPIOx;					// Pointer to GPIO port and registers
	uint8_t pinNumber;						// Pin number (0 - 15)
	GPIO_PinConfig_t config;				// GPIO pin config settings
} GPIO_Handle_t;




/**********************Macros**********************/

/*
 * @GPIO_MODES
 */

#define GPIO_MODE_IN						0x0			// Input
#define GPIO_MODE_OUT						0x1			// Output
#define GPIO_MODE_ALT						0x2			// Alternate function
#define GPIO_MODE_ANALOG					0x3			// Analog
#define GPIO_MODE_IN_RE						0x4			// Input with interrupt triggered on rising edge
#define GPIO_MODE_IN_FE						0x5			// Input with interrupt triggered on falling edge
#define GPIO_MODE_IN_RFE					0x6			// Input with interrupt triggered on rising and falling edge

/*
 * @GPIO_OUTPUT_TYPES
 */

#define GPIO_OT_PP							0x0			// Push-pull
#define GPIO_OT_OD							0x1			// Open-drain

/*
 * @GPIO_SPEEDS
 */

#define GPIO_SPEED_LOW						0x0			// Low speed
#define GPIO_SPEED_MED						0x1			// Medium speed
#define GPIO_SPEED_HI						0x2			// High speed
#define GPIO_SPEED_VHI						0x3			// Very high speed

/*
 * @GPIO_PUPD
 */

#define GPIO_PUPD_NONE						0x0			// No pull-up/pull-down
#define GPIO_PUPD_PU						0x1			// Pull-up
#define GPIO_PUPD_PD						0x2			// Pull-down




/**********************API Prototypes**********************/

/*
 * Validation
 */

int8_t GPIO_Validate_Handle(GPIO_Handle_t* pGPIOHandle);
int8_t GPIO_Validate_Port(GPIO_RegDef_t* pGPIOx);
int8_t GPIO_Validate_IRQ_No(uint8_t IRQNumber);

/*
 * Clock control
 */

int8_t GPIO_Clock(GPIO_RegDef_t* pGPIOx, uint8_t enable);

/*
 * Initialization
 */

int8_t GPIO_InitPin(GPIO_Handle_t* pGPIOHandle);
int8_t GPIO_ResetPort(GPIO_RegDef_t* pGPIOx);

/*
 * Data processing
 */

int8_t GPIO_ReadPin(GPIO_Handle_t* pGPIOHandle);
int32_t GPIO_ReadPort(GPIO_RegDef_t* pGPIOx);
int8_t GPIO_WritePin(GPIO_Handle_t* pGPIOHandle, uint8_t set);
int8_t GPIO_WritePort(GPIO_RegDef_t* pGPIOx, uint16_t val);
int8_t GPIO_TogglePin(GPIO_Handle_t* pGPIOHandle);

/*
 * IRQ Handling
 */

int8_t GPIO_IRQEnable(uint8_t IRQNumber, uint8_t enable);
int8_t GPIO_IRQPriority(uint8_t IRQNumber, uint8_t priority);
int8_t GPIO_IRQClearPending(uint8_t pinNumber);




#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
