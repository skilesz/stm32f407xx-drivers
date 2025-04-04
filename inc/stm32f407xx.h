/*
 * stm32f407xx.h
 *
 *  Created on: Feb 26, 2025
 *      Author: Zach Skiles (skilesz)
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stdbool.h>

#define __vo volatile




/**********************Processor Specific Addresses**********************/

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




/**********************Base Addresses**********************/

/*
 * MEMORY ADDRESSES
 */

#define FLASH_BASE_ADDR					0x08000000U			// Base address of flash memory			(1 MB)
#define ROM_BASE_ADDR					0x1FFF0000U			// Base address of ROM (System Memory)	(30 KB)
#define SRAM1_BASE_ADDR					0x20000000U			// Base address of SRAM1				(112 KB)
#define SRAM2_BASE_ADDR					0x2001C000U			// Base address of SRAM2				(16 KB)
#define SRAM_BASE_ADDR 					SRAM1_BASE_ADDR		// Base address of SRAM memory			(128 KB)

/*
 * BUS ADDRESSES
 */

#define PERIPH_BASE_ADDR				0x40000000U			// Base address of peripheral registers
#define APB1_BASE_ADDR					PERIPH_BASE			// Base address of APB1 peripherals
#define APB2_BASE_ADDR					0x40010000U			// Base address of APB2 peripherals
#define AHB1_BASE_ADDR					0x40020000U			// Base address of AHB1 peripherals
#define AHB2_BASE_ADDR					0x50000000U			// Base address of AHB2 peripherals
#define AHB3_BASE_ADDR					0xA0000000U			// Base address of AHB3 peripherals

/*
 * APB1 PERIPHERAL ADDRESSES
 */

#define TIM2_BASE_ADDR					(APB1_BASE_ADDR + 0x0000)		// TIM2 peripheral
#define TIM3_BASE_ADDR					(APB1_BASE_ADDR + 0x0400)		// Base address of TIM3 peripheral
#define TIM4_BASE_ADDR					(APB1_BASE_ADDR + 0x0800)		// Base address of TIM4 peripheral
#define TIM5_BASE_ADDR					(APB1_BASE_ADDR + 0x0C00)		// Base address of TIM5 peripheral
#define TIM6_BASE_ADDR					(APB1_BASE_ADDR + 0x1000)		// Base address of TIM6 peripheral
#define TIM7_BASE_ADDR					(APB1_BASE_ADDR + 0x1400)		// Base address of TIM7 peripheral
#define TIM12_BASE_ADDR					(APB1_BASE_ADDR + 0x1800)		// Base address of TIM12 peripheral
#define TIM13_BASE_ADDR					(APB1_BASE_ADDR + 0x1C00)		// Base address of TIM13 peripheral
#define TIM14_BASE_ADDR					(APB1_BASE_ADDR + 0x2000)		// Base address of TIM14 peripheral
#define RTC_BASE_ADDR					(APB1_BASE_ADDR + 0x2800)		// Base address of RTC and BKP peripheral
#define WWDG_BASE_ADDR					(APB1_BASE_ADDR + 0x2C00)		// Base address of WWDG peripheral
#define IWDG_BASE_ADDR					(APB1_BASE_ADDR + 0x3000)		// Base address of IWDG peripheral
#define I2S2ext_BASE_ADDR				(APB1_BASE_ADDR + 0x3400)		// Base address of I2S2ext peripheral
#define SPI2_BASE_ADDR					(APB1_BASE_ADDR + 0x3800)		// Base address of SPI2/I2S2 peripheral
#define SPI3_BASE_ADDR					(APB1_BASE_ADDR + 0x3C00)		// Base address of SPI3/I2S3 peripheral
#define I2S3ext_BASE_ADDR				(APB1_BASE_ADDR + 0x4000)		// Base address of I2S3ext peripheral
#define USART2_BASE_ADDR				(APB1_BASE_ADDR + 0x4400)		// Base address of USART2 peripheral
#define USART3_BASE_ADDR				(APB1_BASE_ADDR + 0x4800)		// Base address of USART3 peripheral
#define UART4_BASE_ADDR					(APB1_BASE_ADDR + 0x4C00)		// Base address of UART4 peripheral
#define UART5_BASE_ADDR					(APB1_BASE_ADDR + 0x5000)		// Base address of UART5 peripheral
#define I2C1_BASE_ADDR					(APB1_BASE_ADDR + 0x5400)		// Base address of I2C1 peripheral
#define I2C2_BASE_ADDR					(APB1_BASE_ADDR + 0x5800)		// Base address of I2C2 peripheral
#define I2C3_BASE_ADDR					(APB1_BASE_ADDR + 0x5C00)		// Base address of I2C3 peripheral
#define CAN1_BASE_ADDR					(APB1_BASE_ADDR + 0x6400)		// Base address of CAN1 peripheral
#define CAN2_BASE_ADDR					(APB1_BASE_ADDR + 0x6800)		// Base address of CAN2 peripheral
#define PWR_BASE_ADDR					(APB1_BASE_ADDR + 0x7000)		// Base address of PWR peripheral
#define DAC_BASE_ADDR					(APB1_BASE_ADDR + 0x7400)		// Base address of DAC peripheral
#define UART7_BASE_ADDR					(APB1_BASE_ADDR + 0x7800)		// Base address of UART7 peripheral
#define UART8_BASE_ADDR					(APB1_BASE_ADDR + 0x7C00)		// Base address of UART8 peripheral

/*
 * APB2 PERIPHERAL ADDRESSES
 */

#define TIM1_BASE_ADDR					(APB2_BASE_ADDR + 0x0000)		// Base address of TIM1 peripheral
#define TIM8_BASE_ADDR					(APB2_BASE_ADDR + 0x0400)		// Base address of TIM8 peripheral
#define USART1_BASE_ADDR				(APB2_BASE_ADDR + 0x1000)		// Base address of USART1 peripheral
#define USART6_BASE_ADDR				(APB2_BASE_ADDR + 0x1400)		// Base address of USART6 peripheral
#define ADC_BASE_ADDR					(APB2_BASE_ADDR + 0x2000)		// Base address of ADC1/ADC2/ADC3 peripheral
#define SDIO_BASE_ADDR					(APB2_BASE_ADDR + 0x2C00)		// Base address of SDIO peripheral
#define SPI1_BASE_ADDR					(APB2_BASE_ADDR + 0x3000)		// Base address of SPI1 peripheral
#define SPI4_BASE_ADDR					(APB2_BASE_ADDR + 0x3400)		// Base address of SPI4 peripheral
#define SYSCFG_BASE_ADDR				(APB2_BASE_ADDR + 0x3800)		// Base address of SYSCFG peripheral
#define EXTI_BASE_ADDR					(APB2_BASE_ADDR + 0x3C00)		// Base address of EXTI peripheral
#define TIM9_BASE_ADDR					(APB2_BASE_ADDR + 0x4000)		// Base address of TIM9 peripheral
#define TIM10_BASE_ADDR					(APB2_BASE_ADDR + 0x4400)		// Base address of TIM10 peripheral
#define TIM11_BASE_ADDR					(APB2_BASE_ADDR + 0x4800)		// Base address of TIM11 peripheral
#define SPI5_BASE_ADDR					(APB2_BASE_ADDR + 0x5000)		// Base address of SPI5 peripheral
#define SPI6_BASE_ADDR					(APB2_BASE_ADDR + 0x5400)		// Base address of SPI6 peripheral
#define SAI1_BASE_ADDR					(APB2_BASE_ADDR + 0x5800)		// Base address of SPI6 peripheral
#define LTDC_BASE_ADDR					(APB2_BASE_ADDR + 0x6800)		// Base address of LTDC peripheral

/*
 * AHB1 PERIPHERAL ADDRESSES
 */

#define GPIOA_BASE_ADDR					(AHB1_BASE_ADDR + 0x0000)		// Base address of GPIOA peripheral
#define GPIOB_BASE_ADDR					(AHB1_BASE_ADDR + 0x0400)		// Base address of GPIOB peripheral
#define GPIOC_BASE_ADDR					(AHB1_BASE_ADDR + 0x0800)		// Base address of GPIOC peripheral
#define GPIOD_BASE_ADDR					(AHB1_BASE_ADDR + 0x0C00)		// Base address of GPIOD peripheral
#define GPIOE_BASE_ADDR					(AHB1_BASE_ADDR + 0x1000)		// Base address of GPIOE peripheral
#define GPIOF_BASE_ADDR					(AHB1_BASE_ADDR + 0x1400)		// Base address of GPIOF peripheral
#define GPIOG_BASE_ADDR					(AHB1_BASE_ADDR + 0x1800)		// Base address of GPIOG peripheral
#define GPIOH_BASE_ADDR					(AHB1_BASE_ADDR + 0x1C00)		// Base address of GPIOH peripheral
#define GPIOI_BASE_ADDR					(AHB1_BASE_ADDR + 0x2000)		// Base address of GPIOI peripheral
#define GPIOJ_BASE_ADDR					(AHB1_BASE_ADDR + 0x2400)		// Base address of GPIOJ peripheral
#define GPIOK_BASE_ADDR					(AHB1_BASE_ADDR + 0x2800)		// Base address of GPIOK peripheral
#define CRC_BASE_ADDR					(AHB1_BASE_ADDR + 0x3000)		// Base address of CRC peripheral
#define RCC_BASE_ADDR					(AHB1_BASE_ADDR + 0x3800)		// Base address of RCC peripheral
#define FLASH_INTERFACE_BASE_ADDR		(AHB1_BASE_ADDR + 0x3C00)		// Base address of Flash interface register
#define BKPSRAM_BASE_ADDR				(AHB1_BASE_ADDR + 0x4000)		// Base address of backup SRAM
#define DMA1_BASE_ADDR					(AHB1_BASE_ADDR + 0x6000)		// Base address of DMA1 peripheral
#define DMA2_BASE_ADDR					(AHB1_BASE_ADDR + 0x6400)		// Base address of DMA2 peripheral
#define ETHERNET_BASE_ADDR				(AHB1_BASE_ADDR + 0x8000)		// Base address of Ethernet MAC peripheral
#define DMA2D_BASE_ADDR					(AHB1_BASE_ADDR + 0xB000)		// Base address of DMA2D peripheral
#define OTG_HS_BASE_ADDR				(AHB1_BASE_ADDR + 0x20000)		// Base address of USB OTG HS peripheral

/*
 * AHB2 PERIPHERAL ADDRESSES
 */

#define OTG_FS_BASE_ADDR				(AHB2_BASE_ADDR + 0x0000)		// Base address of USB OTG FS peripheral
#define DCMI_BASE_ADDR					(AHB2_BASE_ADDR + 0x50000)		// Base address of DCMI peripheral
#define CRYP_BASE_ADDR					(AHB2_BASE_ADDR + 0x60000)		// Base address of CRYP peripheral
#define HASH_BASE_ADDR					(AHB2_BASE_ADDR + 0x60400)		// Base address of HASH peripheral
#define RNG_BASE_ADDR					(AHB2_BASE_ADDR + 0x60800)		// Base address of RNG peripheral

/*
 * AHB3 PERIPHERAL ADDRESSES
 */

#define FSMC_BASE_ADDR					(AHB3_BASE_ADDR + 0x0000)		// Base address of FSMC control register




/**********************Peripheral Register Definition Structures**********************/

/*
 * GPIO port registers
 */

typedef struct {
	__vo uint32_t MODER;			// GPIO port mode register													Offset: 0x00
	__vo uint32_t OTYPER;			// GPIO port output type register											Offset: 0x04
	__vo uint32_t OSPEEDR;			// GPIO port output speed register											Offset: 0x08
	__vo uint32_t PUPDR;			// GPIO port pull-up/pull-down register										Offset: 0x0C
	__vo uint32_t IDR;				// GPIO port input data register											Offset: 0x10
	__vo uint32_t ODR;				// GPIO port output data register											Offset: 0x14
	__vo uint32_t BSRR;				// GPIO port bit set/reset register											Offset: 0x18
	__vo uint32_t LCKR;				// GPIO port configuration lock register									Offset: 0x1C
	__vo uint32_t AFR[2];			// GPIO alternate function registers (AFR[0] = AFRL, AFR[1] = AFRH)			Offset: 0x20/0x24
} GPIO_RegDef_t;

/*
 * SPI registers
 */

typedef struct {
	__vo uint32_t CR[2];			// SPI control registers 1 - 2										Offset: 0x00
	__vo uint32_t SR;				// SPI status register												Offset: 0x08
	__vo uint32_t DR;				// SPI data register												Offset: 0x0C
	__vo uint32_t CRCPR;			// SPI CRC polynomial register										Offset: 0x10
	__vo uint32_t RXCRCR;			// SPI RX CRC register												Offset: 0x14
	__vo uint32_t TXCRCR;			// SPI TX CRC register												Offset: 0x18
	__vo uint32_t I2SCFGR;			// SPI I2S configuration register									Offset: 0x1C
	__vo uint32_t I2SPR;			// SPI I2S prescaler register										Offset: 0x20
} SPI_RegDef_t;

/*
 * RCC registers
 */
typedef struct {
	__vo uint32_t CR;				// RCC clock control register											Offset: 0x00
	__vo uint32_t PLLCFGR;			// RCC PLL configuration register										Offset: 0x04
	__vo uint32_t CFGR;				// RCC clock configuration register										Offset: 0x08
	__vo uint32_t CIR;				// RCC clock interrupt register											Offset: 0x0C
	__vo uint32_t AHB1RSTR;			// RCC AHB1 peripheral reset register									Offset: 0x10
	__vo uint32_t AHB2RSTR;			// RCC AHB2 peripheral reset register									Offset: 0x14
	__vo uint32_t AHB3RSTR;			// RCC AHB3 peripheral reset register									Offset: 0x18
	uint32_t RESERVED0;				// !!RESERVED!!	(0x1C)
	__vo uint32_t APB1RSTR;			// RCC APB1 peripheral reset register									Offset: 0x20
	__vo uint32_t APB2RSTR;			// RCC APB2 peripheral reset register									Offset: 0x24
	uint32_t RESERVED1[2];			// !!RESERVED!! (0x28, 0x2C)
	__vo uint32_t AHB1ENR;			// RCC AHB1 peripheral clock enable register							Offset: 0x30
	__vo uint32_t AHB2ENR;			// RCC AHB2 peripheral clock enable register							Offset: 0x34
	__vo uint32_t AHB3ENR;			// RCC AHB3 peripheral clock enable register							Offset: 0x38
	uint32_t RESERVED2;				// !!RESERVED!! (0x3C)
	__vo uint32_t APB1ENR;			// RCC APB1 peripheral clock enable register							Offset: 0x40
	__vo uint32_t APB2ENR;			// RCC APB2 peripheral clock enable register							Offset: 0x44
	uint32_t RESERVED3[2];			// !!RESERVED!! (0x48, 0x4C)
	__vo uint32_t AHB1LPENR;		// RCC AHB1 peripheral clock enable in low power mode register			Offset: 0x50
	__vo uint32_t AHB2LPENR;		// RCC AHB2 peripheral clock enable in low power mode register			Offset: 0x54
	__vo uint32_t AHB3LPENR;		// RCC AHB3 peripheral clock enable in low power mode register			Offset: 0x58
	uint32_t RESERVED4;				// !!RESERVED!! (0x5C)
	__vo uint32_t APB1LPENR;		// RCC APB1 peripheral clock enable in low power mode register			Offset: 0x60
	__vo uint32_t APB2LPENR;		// RCC APB2 peripheral clock enabled in low power mode register			Offset: 0x64
	uint32_t RESERVED5[2];			// !!RESERVED!! (0x68, 0x6C)
	__vo uint32_t BDCR;				// RCC Backup domain control register									Offset: 0x70
	__vo uint32_t CSR;				// RCC clock control & status register									Offset: 0x74
	uint32_t RESERVED6[2];			// !!RESERVED!! (0x78, 0x7C)
	__vo uint32_t SSCGR;			// RCC spread spectrum clock generation register						Offset: 0x80
	__vo uint32_t PLLI2SCFGR;		// RCC PLLI2S configuration register									Offset: 0x84
	__vo uint32_t PLLSAICFGR;		// RCC PLL configuration register										Offset: 0x88
	__vo uint32_t DCKCFGR;			// RCC Dedicated Clock Configuration Register							Offset: 0x8C
} RCC_RegDef_t;

/*
 * EXTI registers
 */

typedef struct {
	__vo uint32_t IMR;				// EXTI interrupt mask register											Offset: 0x00
	__vo uint32_t EMR;				// EXTI event mask register												Offset: 0x04
	__vo uint32_t RTSR;				// EXTI rising trigger selection register								Offset: 0x08
	__vo uint32_t FTSR;				// EXTI falling trigger selection register								Offset: 0x0C
	__vo uint32_t SWIER;			// EXTI software interrupt event register								Offset: 0x10
	__vo uint32_t PR;				// EXTI pending register												Offset: 0x14
} EXTI_RegDef_t;

/*
 * SYSCFG registers
 */

typedef struct {
	__vo uint32_t MEMRMP;			// SYSCFG memory remap register											Offset: 0x00
	__vo uint32_t PMC;				// SYSCFG peripheral mode configuration register						Offset: 0x04
	__vo uint32_t EXTICR[4];		// SYSCFG external interrupt configuration registers 1-4				Offset: 0x08-0x14
	uint32_t RESERVED0[2];			// !!RESERVED!! (0x18, 0x1C)
	__vo uint32_t CMPCR;			// SYSCFG compensation cell control register							Offset: 0x20
} SYSCFG_RegDef_t;




/**********************Peripheral Definitions**********************/

/*
 * GPIO peripherals
 */

#define GPIOA			((GPIO_RegDef_t*) GPIOA_BASE_ADDR)
#define GPIOB			((GPIO_RegDef_t*) GPIOB_BASE_ADDR)
#define GPIOC			((GPIO_RegDef_t*) GPIOC_BASE_ADDR)
#define GPIOD			((GPIO_RegDef_t*) GPIOD_BASE_ADDR)
#define GPIOE			((GPIO_RegDef_t*) GPIOE_BASE_ADDR)
#define GPIOF			((GPIO_RegDef_t*) GPIOF_BASE_ADDR)
#define GPIOG			((GPIO_RegDef_t*) GPIOG_BASE_ADDR)
#define GPIOH			((GPIO_RegDef_t*) GPIOH_BASE_ADDR)
#define GPIOI			((GPIO_RegDef_t*) GPIOI_BASE_ADDR)
#define GPIOJ			((GPIO_RegDef_t*) GPIOJ_BASE_ADDR)
#define GPIOK			((GPIO_RegDef_t*) GPIOK_BASE_ADDR)

/*
 * SPI peripherals
 */

#define SPI1			((SPI_RegDef_t*) SPI1_BASE_ADDR)
#define SPI2			((SPI_RegDef_t*) SPI2_BASE_ADDR)
#define SPI3			((SPI_RegDef_t*) SPI3_BASE_ADDR)
#define SPI4			((SPI_RegDef_t*) SPI4_BASE_ADDR)
#define SPI5			((SPI_RegDef_t*) SPI5_BASE_ADDR)
#define SPI6			((SPI_RegDef_t*) SPI6_BASE_ADDR)


/*
 * RCC peripheral
 */

#define RCC				((RCC_RegDef_t*) RCC_BASE_ADDR)

/*
 * EXTI peripheral
 */

#define EXTI			((EXTI_RegDef_t*) EXTI_BASE_ADDR)

/*
 * SYSCFG peripheral
 */

#define SYSCFG			((SYSCFG_RegDef_t*) SYSCFG_BASE_ADDR)




/**********************Clock Macros**********************/

/*
 * GPIO peripheral clock enable macros
 */

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()			(RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN()			(RCC->AHB1ENR |= (1 << 10))

/*
 * I2C peripheral clock enable macros
 */

#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1 << 23))

/*
 * SPI peripheral clock enable macros
 */

#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()			(RCC->APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN()			(RCC->APB2ENR |= (1 << 21))

/*
 * USART peripheral clock enable macros
 */

#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))

/*
 * SYSCFG peripheral clock enable macros
 */

#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))

/*
 * GPIO peripheral clock disable macros
 */

#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 10))

/*
 * I2C peripheral clock disable macros
 */

#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 23))

/*
 * SPI peripheral clock disable macros
 */

#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 21))

/*
 * USART peripheral clock disable macros
 */

#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5))

/*
 * SYSCFG peripheral clock disable macros
 */

#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))

/*
 * GPIO port reset macros
 */

#define GPIOA_REG_RESET()		do { RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while(0)
#define GPIOB_REG_RESET()		do { RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1); } while(0)
#define GPIOC_REG_RESET()		do { RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2); } while(0)
#define GPIOD_REG_RESET()		do { RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3); } while(0)
#define GPIOE_REG_RESET()		do { RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4); } while(0)
#define GPIOF_REG_RESET()		do { RCC->AHB1RSTR |= (1 << 5); RCC->AHB1RSTR &= ~(1 << 5); } while(0)
#define GPIOG_REG_RESET()		do { RCC->AHB1RSTR |= (1 << 6); RCC->AHB1RSTR &= ~(1 << 6); } while(0)
#define GPIOH_REG_RESET()		do { RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7); } while(0)
#define GPIOI_REG_RESET()		do { RCC->AHB1RSTR |= (1 << 8); RCC->AHB1RSTR &= ~(1 << 8); } while(0)
#define GPIOJ_REG_RESET()		do { RCC->AHB1RSTR |= (1 << 9); RCC->AHB1RSTR &= ~(1 << 9); } while(0)
#define GPIOK_REG_RESET()		do { RCC->AHB1RSTR |= (1 << 10); RCC->AHB1RSTR &= ~(1 << 10); } while(0)

/*
 * GPIO port code macro
 */

#define GPIO_PORT_TO_CODE(x)	((x == GPIOA) ? 0x0 :\
								 (x == GPIOB) ? 0x1 :\
								 (x == GPIOC) ? 0x2 :\
								 (x == GPIOD) ? 0x3 :\
								 (x == GPIOE) ? 0x4 :\
								 (x == GPIOF) ? 0x5 :\
								 (x == GPIOG) ? 0x6 :\
								 (x == GPIOH) ? 0x7 : 0x8)

/*
 * @IRQ_Numbers
 */

#define IRQ_EXTI0				6
#define IRQ_EXTI1				7
#define IRQ_EXTI2				8
#define IRQ_EXTI3				9
#define IRQ_EXTI4				10
#define IRQ_EXTI9_5				23
#define IRQ_EXTI15_10			40


/*
 * Generic macros
 */

#define ENABLE					1
#define SET						1

#define DISABLE					0
#define RESET					0




/**********************Driver Includes**********************/

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"




#endif /* INC_STM32F407XX_H_ */
