/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Feb 28, 2025
 *      Author: Zach Skiles (skilesz)
 */


#include "stm32f407xx_gpio_driver.h"




/**********************Function Definitions**********************/

/*
 * @fn				GPIO_Clock
 *
 * @desc			Enables or disables peripheral clock for given GPIO port
 *
 * @param			pGPIOx: base address of GPIO port
 * @param			enable: ENABLE or DISABLE macros
 *
 * @return			void
 */
void GPIO_Clock(GPIO_RegDef_t* pGPIOx, uint8_t enable) {

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
			break;
	}
}

/*
 * @fn				GPIO_InitPin
 *
 * @desc			Initializes pin with specified properties
 *
 * @param			pGPIOHandle: settings of pin to be configured
 *
 * @return			void
 */
void GPIO_InitPin(GPIO_Handle_t* pGPIOHandle) {

	// Configure mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		pGPIOHandle->pGPIOx->MODER &=
			~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
		pGPIOHandle->pGPIOx->MODER |=
			(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
	} else {

	}

	// Configure output type
	pGPIOHandle->pGPIOx->OTYPER &=
		~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |=
		(pGPIOHandle->GPIO_PinConfig.GPIO_PinOutputType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	// Configure speed
	pGPIOHandle->pGPIOx->OSPEEDR &=
		~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
	pGPIOHandle->pGPIOx->OSPEEDR |=
		(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));

	// Configure PUPD settings
	pGPIOHandle->pGPIOx->PUPDR &=
		~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
	pGPIOHandle->pGPIOx->PUPDR |=
		(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPd << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));

	// Configure alternate function mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT) {
		uint8_t afrlh = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode / 8;
		uint8_t afrPos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[afrlh] &=
			~(0xf << ((afrPos) * 4));
		pGPIOHandle->pGPIOx->AFR[afrlh] |=
			(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ((afrPos) * 4));
	}
}

/*
 * @fn				GPIO_ResetPort
 *
 * @desc			Resets specified port
 *
 * @param			pGPIOx: base address of GPIO port
 *
 * @return			void
 */
void GPIO_ResetPort(GPIO_RegDef_t* pGPIOx){

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
			break;
	}
}

/*
 * @fn				GPIO_ReadPin
 *
 * @desc			Reads specified pin
 *
 * @param			pGPIOHandle: handle of pin to be read from
 *
 * @return			Value of the specified pin
 */
uint8_t GPIO_ReadPin(GPIO_Handle_t* pGPIOHandle) {
	uint8_t value;
	value = (uint8_t) (pGPIOHandle->pGPIOx->IDR >> pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) & 0x1;

	return value;
}

/*
 * @fn				GPIO_ReadPort
 *
 * @desc			Reads all pins of specified port
 *
 * @param			pGPIOx: base address of GPIO port
 *
 * @return			Value of all pins of specified port
 */
uint16_t GPIO_ReadPort(GPIO_RegDef_t* pGPIOx) {
	return (uint16_t) pGPIOx->IDR;
}

/*
 * @fn				GPIO_WritePin
 *
 * @desc			Writes value to specified pin
 *
 * @param			pGPIOHandle: handle of pin to be written to
 * @param 			set: SET or RESET macros
 *
 * @return			void
 */
void GPIO_WritePin(GPIO_Handle_t* pGPIOHandle, uint8_t set) {
	if (set == SET) pGPIOHandle->pGPIOx->BSRR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	else pGPIOHandle->pGPIOx->BSRR |= (1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber + 16));
}

/*
 * @fn				GPIO_WritePort
 *
 * @desc			Writes value to specified port
 *
 * @param			pGPIOx: base address of GPIO port
 * @param			val: value to be written to each pin
 *
 * @return			void
 */
void GPIO_WritePort(GPIO_RegDef_t* pGPIOx, uint16_t val) {
	pGPIOx->ODR = val;
}

/*
 * @fn				GPIO_TogglePin
 *
 * @desc			Toggle specified pin
 *
 * @param			pGPIOHandle: handle of the pin to be toggled
 *
 * @return			void
 */
void GPIO_TogglePin(GPIO_Handle_t* pGPIOHandle) {
	pGPIOHandle->pGPIOx->ODR ^= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
}

/*
 * @fn				GPIO_IRQConfig
 *
 * @desc			Configures specified IRQ
 *
 * @param			IRQNumber: number of IRQ
 * @param			IRQPriority: priority of IRQ
 * @param			enable: ENABLE or DISABLE macros
 *
 * @return			void
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t enable) {

}

/*
 * @fn				GPIO_IRQHandle
 *
 * @desc			Handles interrupt for specified pin
 *
 * @param			pinNumber: number of pin who's IRQ will be handled
 *
 * @return			void
 */
void GPIO_IRQHandle(uint8_t pinNumber){

}
