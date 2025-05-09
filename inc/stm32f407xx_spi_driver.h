/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Mar 26, 2025
 *      Author: Zach Skiles (skilesz)
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"




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




/**********************Macros**********************/

/*
 * @SPI_MODES
 */

#define SPI_MODE_MASTER						0x0			// Master
#define SPI_MODE_SLAVE						0x1			// Slave


/*
 * @SPI_BUS_CONFIGS
 */

#define SPI_BUS_FD							0x0			// Full-duplex
#define SPI_BUS_HD							0x1			// Half-duplex
#define SPI_BUS_SIRX						0x2			// Simplex (RX only)

/*
 * @SPI_SPEEDS
 */

#define SPI_SPEED_DIV2						0x0			// PCLK / 2
#define SPI_SPEED_DIV4						0x1			// PCLK / 4
#define SPI_SPEED_DIV8						0x2			// PCLK / 8
#define SPI_SPEED_DIV16						0x3			// PCLK / 16
#define SPI_SPEED_DIV32						0x4			// PCLK / 32
#define SPI_SPEED_DIV64						0x5			// PCLK / 64
#define SPI_SPEED_DIV128					0x6			// PCLK / 128
#define SPI_SPEED_DIV256					0x7			// PCLK / 256

/*
 * @SPI_DFF
 */

#define SPI_DFF_8BIT						0x0			// 8-bit data
#define SPI_DFF_16BIT						0x1			// 16-bit data

/*
 * @SPI_CPOL
 */

#define SPI_CPOL_0							0x0			// CLK to 0 when idle
#define SPI_CPOL_1							0x1			// CLK to 1 when idle

/*
 * @SPI_CPHA
 */

#define SPI_CPHA_1ST						0x0			// Data capture on first clock edge
#define SPI_CPHA_2ND						0x1			// Data capture on second clock edge

/*
 * @SPI_SSM
 */

#define SPI_SSM_DI							0x0			// Software slave management disabled
#define SPI_SSM_EN							0x1			// Software slave management enabled

/*
 * Bit position macros
 */

#define SPI_CR1_CPHA			0x0
#define SPI_CR1_CPOL			0x1
#define SPI_CR1_MSTR			0x2
#define SPI_CR1_BR				0x3
#define SPI_CR1_SPE				0x6
#define SPI_CR1_LSBFIRST		0x7
#define SPI_CR1_SSI				0x8
#define SPI_CR1_SSM				0x9
#define SPI_CR1_RXONLY			0xA
#define SPI_CR1_DFF				0xB
#define SPI_CR1_CRCNEXT			0xC
#define SPI_CR1_CRCEN			0xD
#define SPI_CR1_BDIOE			0xE
#define SPI_CR1_BIDIMODE		0xF

#define SPI_CR2_RXDMAEN			0x0
#define SPI_CR2_TXDMAEN			0x1
#define SPI_CR2_SSOE			0x2
#define SPI_CR2_FRF				0x4
#define SPI_CR2_ERRIE			0x5
#define SPI_CR2_RXNEIE			0x6
#define SPI_CR2_TXEIE			0x7

#define SPI_SR_RXNE				0x0
#define SPI_SR_TXE				0x1
#define SPI_SR_CHSIDE			0x2
#define SPI_SR_UDR				0x3
#define SPI_SR_CRCERR			0x4
#define SPI_SR_MODF				0x5
#define SPI_SR_OVR				0x6
#define SPI_SR_BSY				0x7
#define SPI_SR_FRE				0x8




/**********************API Prototypes**********************/

/*
 * Validation
 */

int8_t SPI_Validate_Handle(SPI_Handle_t* pSPIHandle);
int8_t SPI_Validate_Pointer(SPI_RegDef_t* pSPIx);
int8_t SPI_Validate_IRQ_No(uint8_t IRQNumber);

/*
 * Clock control
 */

int8_t SPI_Clock(SPI_RegDef_t* pSPIx, uint8_t enable);

/*
 * Initialization
 */

int8_t SPI_Init(SPI_Handle_t* pSPIHandle);
int8_t SPI_Reset(SPI_RegDef_t* pSPIx);

/*
 * Data send/receive
 */

int8_t SPI_Send(SPI_RegDef_t* pSPIx, uint8_t* pData, uint32_t len);
int8_t SPI_Receive(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len);

/*
 * IRQ Handling
 */

int8_t SPI_IRQEnable(uint8_t IRQNumber, uint8_t enable);
int8_t SPI_IRQPriority(uint8_t IRQNumber, uint8_t priority);
int8_t SPI_IRQClearPending(SPI_Handle_t* pSPIHandle);


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
