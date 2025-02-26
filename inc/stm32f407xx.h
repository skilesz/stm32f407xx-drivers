/*
 * stm32f407xx.h
 *
 *  Created on: Feb 26, 2025
 *      Author: Zach Skiles (skilesz)
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_



/*
 * MEMORY ADDRESSES
 */

#define FLASH_BASE_ADDR					0x08000000U			// Base address of flash memory 		(1 MB)
#define ROM_BASE_ADDR					0x1FFF0000U			// Base address of ROM (System Memory) 	(30 KB)
#define SRAM1_BASE_ADDR					0x20000000U			// Base address of SRAM1 				(112 KB)
#define SRAM2_BASE_ADDR					0x2001C000U			// Base address of SRAM2 				(16 KB)
#define SRAM_BASE_ADDR 					SRAM1_BASE_ADDR		// Base address of SRAM memory 			(128 KB)

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



#endif /* INC_STM32F407XX_H_ */
