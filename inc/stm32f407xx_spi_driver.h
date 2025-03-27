/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Mar 26, 2025
 *      Author: Zach Skiles (skilesz)
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_




/**********************Structures**********************/

/*
 * Config structure
 */

typedef struct {
	uint8_t mode;					// Device mode (@SPI_MODES)
	uint8_t busConfig;				// Bus configuration (@SPI_BUS_CONFIGS)
	uint8_t speed;					// SCLK speed (@SPI_SPEEDS)
	uint8_t DFF;					// Data frame format (@SPI_DFF)
	uint8_t CPOL;					// Clock polarity (@SPI_CPOL)
	uint8_t CPHA;					// Clock phase (@SPI_CPHA)
	uint8_t SSM;					// Software slave management (@SPI_SSM)
} SPI_Config_t;

/*
 * Handle structure
 */

typedef struct {
	SPI_RegDef_t* pSPIx;				// Pointer to SPI peripheral
	SPI_Config_t config;				// SPI config settings
} SPI_Handle_t;




#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
