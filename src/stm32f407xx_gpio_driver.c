/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Feb 28, 2025
 *      Author: Zach Skiles (skilesz)
 */


#include "stm32f407xx_gpio_driver.h"




/**********************Function Definitions**********************/

/*
 * @ERROR_CODES
 *
 * -1	-> null port pointer
 * -2	-> invalid port pointer
 * -3	-> null handle pointer
 * -4	-> invalid pin number
 * -5	-> invalid mode
 * -6	-> invalid output type
 * -7	-> invalid speed
 * -8	-> invalid pull-up/pull-down configuration
 * -9	-> invalid alternate function
 * -10	-> invalid enable value
 * -11	-> specified pin does not support interrupt mode
 * -12	-> invalid set value
 * -13	-> invalid IRQ number
 * -14	-> invalid IRQ priority
 */

/*
 * @fn				GPIO_Validate_Handle
 *
 * @desc			Validates a given GPIO handle
 *
 * @param			pGPIOHandle: handle of a GPIO pin
 *
 * @return			0			-> valid handle
 * 					NEGATIVE	-> see @ERROR_CODES
 */

int8_t	GPIO_Validate_Handle(GPIO_Handle_t* pGPIOHandle) {
	if (!pGPIOHandle) return -3;

	int8_t portValidate = GPIO_Validate_Port(pGPIOHandle->pGPIOx);
	if (portValidate) return portValidate;

	if (!(pGPIOHandle->pinNumber >= 0 && pGPIOHandle->pinNumber <= 15)) return -4;
	if (!(pGPIOHandle->config.mode >= GPIO_MODE_IN && pGPIOHandle->config.mode <= GPIO_MODE_IN_RFE)) return -5;
	if (!(pGPIOHandle->config.outputType >= GPIO_OT_PP && pGPIOHandle->config.outputType <= GPIO_OT_OD)) return -6;
	if (!(pGPIOHandle->config.speed >= GPIO_SPEED_LOW && pGPIOHandle->config.speed <= GPIO_SPEED_VHI)) return -7;
	if (!(pGPIOHandle->config.pupd >= GPIO_PUPD_NONE && pGPIOHandle->config.pupd <= GPIO_PUPD_PD)) return -8;
	if (!(pGPIOHandle->config.altFun >= 0 && pGPIOHandle->config.altFun <= 15)) return -9;

	return 0;
}

/*
 * @fn				GPIO_Validate_Port
 *
 * @desc			Validates a given port
 *
 * @param			pGPIOx: GPIO port address
 *
 * @return			0			-> valid port
 * 					NEGATIVE	-> see @ERROR_CODES
 */
int8_t GPIO_Validate_Port(GPIO_RegDef_t* pGPIOx) {
	if (!pGPIOx) return -1;

	switch((uint32_t) pGPIOx) {
		case (uint32_t) GPIOA:
		case (uint32_t) GPIOB:
		case (uint32_t) GPIOC:
		case (uint32_t) GPIOD:
		case (uint32_t) GPIOE:
		case (uint32_t) GPIOF:
		case (uint32_t) GPIOG:
		case (uint32_t) GPIOH:
		case (uint32_t) GPIOI:
		case (uint32_t) GPIOJ:
		case (uint32_t) GPIOK:
			break;
		default:
			return -2;
	}

	return 0;
}

/*
 * @fn 				GPIO_Validate_IRQ_No
 *
 * @desc 			Validates a given IRQ number
 *
 * @param			IRQNumber: the number of the IRQ to be configured
 *
 * @return			0			-> valid IRQ number
 * 					NEGATIVE	-> see @ERROR_CODES
 */

int8_t GPIO_Validate_IRQ_No(uint8_t IRQNumber) {
	switch(IRQNumber) {
		case IRQ_EXTI0:
		case IRQ_EXTI1:
		case IRQ_EXTI2:
		case IRQ_EXTI3:
		case IRQ_EXTI4:
		case IRQ_EXTI9_5:
		case IRQ_EXTI15_10:
			break;
		default:
			return -13;
	}

	return 0;
}

/*
 * @fn				GPIO_Clock
 *
 * @desc			Enables or disables peripheral clock for given GPIO port
 *
 * @param			pGPIOx: base address of GPIO port
 * @param			enable: ENABLE or DISABLE macros
 *
 * @return			0			-> success
 *					NEGATIVE	-> see @ERROR_CODES
 */
int8_t GPIO_Clock(GPIO_RegDef_t* pGPIOx, uint8_t enable) {
	// Error checks
	if (!(enable == ENABLE || enable == DISABLE)) return -10;

	// Enable/disable port
	switch ((uint32_t) pGPIOx) {
		case (uint32_t) GPIOA:
			if (enable == ENABLE) GPIOA_PCLK_EN();
			else GPIOA_PCLK_DI();
			break;
		case (uint32_t) GPIOB:
			if (enable == ENABLE) GPIOB_PCLK_EN();
			else GPIOB_PCLK_DI();
			break;
		case (uint32_t) GPIOC:
			if (enable == ENABLE) GPIOC_PCLK_EN();
			else GPIOC_PCLK_DI();
			break;
		case (uint32_t) GPIOD:
			if (enable == ENABLE) GPIOD_PCLK_EN();
			else GPIOD_PCLK_DI();
			break;
		case (uint32_t) GPIOE:
			if (enable == ENABLE) GPIOE_PCLK_EN();
			else GPIOE_PCLK_DI();
			break;
		case (uint32_t) GPIOF:
			if (enable == ENABLE) GPIOF_PCLK_EN();
			else GPIOF_PCLK_DI();
			break;
		case (uint32_t) GPIOG:
			if (enable == ENABLE) GPIOG_PCLK_EN();
			else GPIOG_PCLK_DI();
			break;
		case (uint32_t) GPIOH:
			if (enable == ENABLE) GPIOH_PCLK_EN();
			else GPIOH_PCLK_DI();
			break;
		case (uint32_t) GPIOI:
			if (enable == ENABLE) GPIOI_PCLK_EN();
			else GPIOI_PCLK_DI();
			break;
		case (uint32_t) GPIOJ:
			if (enable == ENABLE) GPIOJ_PCLK_EN();
			else GPIOJ_PCLK_DI();
			break;
		case (uint32_t) GPIOK:
			if (enable == ENABLE) GPIOK_PCLK_EN();
			else GPIOK_PCLK_DI();
			break;
		default:
			return -2;
	}

	return 0;
}

/*
 * @fn				GPIO_InitPin
 *
 * @desc			Initializes pin with specified properties
 *
 * @param			pGPIOHandle: settings of pin to be configured
 *
 * @return			0			-> success
 * 					NEGATIVE	-> see @ERROR_CODES
 */
int8_t GPIO_InitPin(GPIO_Handle_t* pGPIOHandle) {
	// Error checks
	int8_t handleValidate = GPIO_Validate_Handle(pGPIOHandle);
	if (handleValidate) return handleValidate;

	// Configure mode
	if (pGPIOHandle->config.mode <= GPIO_MODE_ANALOG) {
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (pGPIOHandle->pinNumber * 2));
		pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->config.mode << (pGPIOHandle->pinNumber * 2));
	} else {
		// Configure input trigger modes

		// Check for valid trigger pin
		if (pGPIOHandle->pGPIOx == GPIOK || pGPIOHandle->pGPIOx == GPIOJ ||
			(pGPIOHandle->pGPIOx == GPIOI && pGPIOHandle->pinNumber == 15)) {
			return -11;
		}

		// Configure rising/falling-edge triggers
		switch(pGPIOHandle->config.mode) {
			case GPIO_MODE_IN_FE:
				EXTI->FTSR |= (1 << pGPIOHandle->pinNumber);
				EXTI->RTSR &= ~(1 << pGPIOHandle->pinNumber);
				break;
			case GPIO_MODE_IN_RE:
				EXTI->RTSR |= (1 << pGPIOHandle->pinNumber);
				EXTI->FTSR &= ~(1 << pGPIOHandle->pinNumber);
				break;
			case GPIO_MODE_IN_RFE:
				EXTI->RTSR |= (1 << pGPIOHandle->pinNumber);
				EXTI->FTSR |= (1 << pGPIOHandle->pinNumber);
				break;
		}

		// Enable EXTI interrupt in SYSCFG
		uint8_t exticr = pGPIOHandle->pinNumber / 4;
		uint8_t extipos = pGPIOHandle->pinNumber % 4;
		uint8_t val = GPIO_PORT_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[exticr] &= ~(0xF << (extipos * 4));
		SYSCFG->EXTICR[exticr] |= (val << (extipos * 4));

		// Unmask interrupt
		EXTI->IMR |= (1 << pGPIOHandle->pinNumber);
	}

	// Configure output type
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->pinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= (pGPIOHandle->config.outputType << pGPIOHandle->pinNumber);

	// Configure speed
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (pGPIOHandle->pinNumber * 2));
	pGPIOHandle->pGPIOx->OSPEEDR |= (pGPIOHandle->config.speed << (pGPIOHandle->pinNumber * 2));

	// Configure PUPD settings
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (pGPIOHandle->pinNumber * 2));
	pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->config.pupd << (pGPIOHandle->pinNumber * 2));

	// Configure alternate function mode if applicable
	if (pGPIOHandle->config.mode == GPIO_MODE_ALT) {
		uint8_t afrlh = pGPIOHandle->config.altFun / 8;
		uint8_t afrPos = pGPIOHandle->pinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[afrlh] &= ~(0xf << ((afrPos) * 4));
		pGPIOHandle->pGPIOx->AFR[afrlh] |= (pGPIOHandle->config.altFun << ((afrPos) * 4));
	}

	return 0;
}

/*
 * @fn				GPIO_ResetPort
 *
 * @desc			Resets specified port
 *
 * @param			pGPIOx: base address of GPIO port
 *
 * @return			0			-> success
 * 					NEGATIVE	-> see @ERROR_CODES
 */
int8_t GPIO_ResetPort(GPIO_RegDef_t* pGPIOx){
	// Error checks
	if (!pGPIOx) return -1;

	// Reset port
	switch ((uint32_t) pGPIOx) {
		case (uint32_t) GPIOA:
			GPIOA_REG_RESET();
			break;
		case (uint32_t) GPIOB:
			GPIOB_REG_RESET();
			break;
		case (uint32_t) GPIOC:
			GPIOC_REG_RESET();
			break;
		case (uint32_t) GPIOD:
			GPIOD_REG_RESET();
			break;
		case (uint32_t) GPIOE:
			GPIOE_REG_RESET();
			break;
		case (uint32_t) GPIOF:
			GPIOF_REG_RESET();
			break;
		case (uint32_t) GPIOG:
			GPIOG_REG_RESET();
			break;
		case (uint32_t) GPIOH:
			GPIOH_REG_RESET();
			break;
		case (uint32_t) GPIOI:
			GPIOI_REG_RESET();
			break;
		case (uint32_t) GPIOJ:
			GPIOJ_REG_RESET();
			break;
		case (uint32_t) GPIOK:
			GPIOK_REG_RESET();
			break;
		default:
			return -2;
	}

	return 0;
}

/*
 * @fn				GPIO_ReadPin
 *
 * @desc			Reads specified pin
 *
 * @param			pGPIOHandle: handle of pin to be read from
 *
 * @return			0x0 - 0x1	-> success
 * 					NEGATIVE	-> see @ERROR_CODES
 */
int8_t GPIO_ReadPin(GPIO_Handle_t* pGPIOHandle) {
	// Error checks
	int8_t handleValidate = GPIO_Validate_Handle(pGPIOHandle);
	if (handleValidate) return handleValidate;

	// Read pin from port input register
	int8_t pinVal = (int8_t) (pGPIOHandle->pGPIOx->IDR >> pGPIOHandle->pinNumber) & 0x1;

	return pinVal;
}

/*
 * @fn				GPIO_ReadPort
 *
 * @desc			Reads all pins of specified port
 *
 * @param			pGPIOx: base address of GPIO port
 *
 * @return			0x0 - 0xFFFF	-> success
 * 					NEGATIVE		-> see @ERROR_CODES
 */
int32_t GPIO_ReadPort(GPIO_RegDef_t* pGPIOx) {
	// Error checks
	int8_t portValidate = GPIO_Validate_Port(pGPIOx);
	if (portValidate) return portValidate;

	// Read port from input register
	int32_t portVal = (int32_t) pGPIOx->IDR;

	return portVal;
}

/*
 * @fn				GPIO_WritePin
 *
 * @desc			Writes value to specified pin
 *
 * @param			pGPIOHandle: handle of pin to be written to
 * @param 			set: SET or RESET macros
 *
 * @return			0			-> success
 * 					NEGATIVE	-> see @ERROR_CODES
 */
int8_t GPIO_WritePin(GPIO_Handle_t* pGPIOHandle, uint8_t set) {
	// Error checks
	int8_t handleValidate = GPIO_Validate_Handle(pGPIOHandle);
	if (handleValidate) return handleValidate;
	if (!(set == SET || set == RESET)) return -12;

	// Write to pin with output set/reset registers
	if (set == SET) pGPIOHandle->pGPIOx->BSRR |= (1 << pGPIOHandle->pinNumber);
	else pGPIOHandle->pGPIOx->BSRR |= (1 << (pGPIOHandle->pinNumber + 16));

	return 0;
}

/*
 * @fn				GPIO_WritePort
 *
 * @desc			Writes value to specified port
 *
 * @param			pGPIOx: base address of GPIO port
 * @param			val: value to be written to each pin
 *
 * @return			0			-> success
 * 					NEGATIVE	-> see @ERROR_CODES
 */
int8_t GPIO_WritePort(GPIO_RegDef_t* pGPIOx, uint16_t val) {
	// Error checks
	int8_t portValidate = GPIO_Validate_Port(pGPIOx);
	if (portValidate) return portValidate;

	// Write to port with output register
	pGPIOx->ODR = val;

	return 0;
}

/*
 * @fn				GPIO_TogglePin
 *
 * @desc			Toggle specified pin
 *
 * @param			pGPIOHandle: handle of the pin to be toggled
 *
 * @return			0			-> success
 * 					NEGATIVE	-> see @ERROR_CODES
 */
int8_t GPIO_TogglePin(GPIO_Handle_t* pGPIOHandle) {
	// Error checks
	int8_t handleValidate = GPIO_Validate_Handle(pGPIOHandle);
	if (handleValidate) return handleValidate;

	// Toggle pin with port output register
	pGPIOHandle->pGPIOx->ODR ^= (1 << pGPIOHandle->pinNumber);

	return 0;
}

/*
 * @fn				GPIO_IRQEnable
 *
 * @desc			Enables specified IRQ
 *
 * @param			IRQNumber: number of IRQ
 * @param			enable: ENABLE or DISABLE macros
 *
 * @return			0			-> success
 * 					NEGATIVE	-> see @ERROR_CODES
 */
int8_t GPIO_IRQEnable(uint8_t IRQNumber, uint8_t enable) {
	// Error checks
	int8_t IRQNoValidate = GPIO_Validate_IRQ_No(IRQNumber);
	if (IRQNoValidate) return IRQNoValidate;
	if (!(enable == ENABLE || enable == DISABLE)) return -10;

	// Enable or disable IRQ in NVIC registers
	uint8_t nvicreg = IRQNumber / 32;
	uint8_t nvicpos	= IRQNumber % 32;

	if (enable == ENABLE) NVIC_ISER[nvicreg] |= (1 << nvicpos);
	else NVIC_ICER[nvicreg] |= (1 << nvicpos);

	return 0;
}

/*
 * @fn 				GPIO_IRQPriority
 *
 * @desc 			Sets the priority for given IRQ number
 *
 * @param			IRQNumber: number of IRQ (see @IRQ_Numbers)
 * @param			priority: priority to be set (0 - 15)
 *
 * @return			0			-> success
 * 					NEGATIVE	-> see @ERROR_CODES
 */

int8_t GPIO_IRQPriority(uint8_t IRQNumber, uint8_t priority) {
	// Error checks
	int8_t IRQNoValidate = GPIO_Validate_IRQ_No(IRQNumber);
	if (IRQNoValidate) return IRQNoValidate;
	if (!(priority >= 0 && priority <= 15)) return -14;

	// Set corresponding IRQ priority
	uint8_t iprreg = IRQNumber / 4;
	uint8_t iprpos = IRQNumber % 4;

	// Set priority
	NVIC_IPR[iprreg] &= ~(0xF << ((iprpos * 8) + 4));
	NVIC_IPR[iprreg] |= (priority << ((iprpos * 8) + 4));

	return 0;
}

/*
 * @fn				GPIO_IRQClearPending
 *
 * @desc			Clears interrupt pending bit for specified pin
 *
 * @param			pinNumber: number of pin that triggered interrupt
 * @param
 *
 * @return			0			-> success
 * 					NEGATIVE	-> see @ERROR_CODES
 */
int8_t GPIO_IRQClearPending(uint8_t pinNumber){
	// Error checks
	if (!(pinNumber >= 0 && pinNumber <= 15)) return -4;

	// Clear interrupt pending bit
	if (EXTI->PR & (1 << pinNumber)) EXTI->PR |= (1 << pinNumber);

	return 0;
}
