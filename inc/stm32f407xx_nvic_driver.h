/*
 * stm32f407xx_nvic_driver.h
 *
 *  Created on: Sep 9, 2025
 *      Author: Zach Skiles (skilesz)
 */

#ifndef INC_STM32F407XX_NVIC_DRIVER_H_
#define INC_STM32F407XX_NVIC_DRIVER_H_

#include "stm32f407xx.h"




/**********************Macros**********************/

/*
 * NVIC registers
 */

#define NVIC_ISER					((uint32_t*) 0xE000E100U)		// Base address of NVIC_ISER registers
#define NVIC_ICER					((uint32_t*) 0xE000E180U)		// Base address of NVIC_ICER registers
#define NVIC_ISPR					((uint32_t*) 0xE000E200U)		// Base address of NVIC_ISPR registers
#define NVIC_ICPR					((uint32_t*) 0xE000E280U)		// Base address of NVIC_ICPR registers
#define NVIC_IABR					((uint32_t*) 0xE000E300U)		// Base address of NVIC_IABR registers
#define NVIC_IPR					((uint32_t*) 0xE000E400U)		// Base address of NVIC_IPR registers
#define NVIC_STIR					((uint32_t*) 0xE000EF00U)		// Base address of NVIC_STIR register




#endif /* INC_STM32F407XX_NVIC_DRIVER_H_ */
