/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Mar 26, 2025
 *      Author: Zach Skiles (skilesz)
 */


#include "stm32f407xx_spi_driver.h"




/**********************Function Definitions**********************/

/*
 * @ERROR_CODES
 *
 * -1	-> null SPI pointer
 * -2	-> invalid SPI pointer
 * -3	-> null SPI handle
 * -4	-> invalid mode
 * -5	-> invalid bus configuration
 * -6	-> invalid speed (baud rate)
 * -7	-> invalid data frame format
 * -8	-> invalid clock polarity
 * -9	-> invalid clock phase
 * -10	-> invalid software slave management configuration
 * -11	-> invalid enable value
 * -12	-> invalid IRQ number
 * -13	-> null TX data pointer
 * -14	-> invalid length
 * -15	-> invalid length for 16-bit data frame (must be divisible by 2)
 */

/*
 * @fn				SPI_Validate_Handle
 *
 * @desc			Validates a given SPI handle
 *
 * @param			pSPIHandle: handle of a given SPI configuration
 *
 * @return			0			-> valid handle
 * 					NEGATIVE	-> see @ERROR_CODES
 */

int8_t SPI_Validate_Handle(SPI_Handle_t* pSPIHandle) {
	if (!pSPIHandle) return -3;

	int8_t pointerValidate = SPI_Validate_Pointer(pSPIHandle->pSPIx);
	if (pointerValidate) return pointerValidate;

	if (!(pSPIHandle->config.mode >= SPI_MODE_MASTER && pSPIHandle->config.mode <= SPI_MODE_SLAVE)) return -4;
	if (!(pSPIHandle->config.busConfig >= SPI_BUS_FD && pSPIHandle->config.busConfig <= SPI_BUS_SIRX)) return -5;
	if (!(pSPIHandle->config.speed >= SPI_SPEED_DIV2 && pSPIHandle->config.speed <= SPI_SPEED_DIV256)) return -6;
	if (!(pSPIHandle->config.DFF >= SPI_DFF_8BIT && pSPIHandle->config.DFF <= SPI_DFF_16BIT)) return -7;
	if (!(pSPIHandle->config.CPOL >= SPI_CPOL_0 && pSPIHandle->config.CPOL <= SPI_CPOL_1)) return -8;
	if (!(pSPIHandle->config.CPHA >= SPI_CPHA_1ST && pSPIHandle->config.CPHA <= SPI_CPHA_2ND)) return -9;
	if (!(pSPIHandle->config.SSM >= SPI_SSM_DI && pSPIHandle->config.SSM <= SPI_SSM_EN)) return -10;

	return 0;
}

/*
 * @fn				SPI_Validate_Pointer
 *
 * @desc			Validates a given SPI pointer
 *
 * @param			pSPIx: pointer to SPI peripheral
 *
 * @return			0			-> valid pointer
 * 					NEGATIVE	-> see @ERROR_CODES
 */

int8_t SPI_Validate_Pointer(SPI_RegDef_t* pSPIx) {
	if (!pSPIx) return -1;

	switch ((uint32_t) pSPIx) {
		case (uint32_t) SPI1:
		case (uint32_t) SPI2:
		case (uint32_t) SPI3:
			break;
		default:
			return -2;
	}

	return 0;
}

/*
 * @fn				SPI_Validate_IRQ_No
 *
 * @desc			Validates given IRQ number
 *
 * @param			IRQNumber: the number of the IRQ to be validated
 *
 * @return			0			-> valid IRQ number
 * 					NEGATIVE	-> see @ERROR_CODES
 */

int8_t SPI_Validate_IRQ_No(uint8_t IRQNumber) {
	switch (IRQNumber) {
		case IRQ_SPI1:
		case IRQ_SPI2:
		case IRQ_SPI3:
			break;
		default:
			return -12;
	}

	return 0;
}

/*
 * @fn				SPI_Clock
 *
 * @desc			Enables or disables clock for given SPI peripheral
 *
 * @param			pSPIx: pointer to SPI peripheral
 * 					enable: ENABLE or DISABLE macros
 *
 * @return			0			-> success
 * 					NEGATIVE	-> see @ERROR_CODES
 */

int8_t SPI_Clock(SPI_RegDef_t* pSPIx, uint8_t enable) {
	// Error checks
	if (!pSPIx) return -1;
	if (!(enable == ENABLE || enable == DISABLE)) return -11;

	// Enable/disable peripheral clock
	switch ((uint32_t) pSPIx) {
		case (uint32_t) SPI1:
			if (enable == ENABLE) SPI1_PCLK_EN();
			else SPI1_PCLK_DI();
			break;
		case (uint32_t) SPI2:
			if (enable == ENABLE) SPI2_PCLK_EN();
			else SPI2_PCLK_DI();
			break;
		case (uint32_t) SPI3:
			if (enable == ENABLE) SPI3_PCLK_EN();
			else SPI3_PCLK_DI();
			break;
		default:
			return -2;
	}

	return 0;
}

/*
 * @fn				SPI_Init
 *
 * @desc			Initializes SPI peripheral with specified configuration
 *
 * @param			pSPIHandle: settings of peripheral to be configured
 *
 * @return			0			-> success
 * 					NEGATIVE	-> see @ERROR_CODES
 */

int8_t SPI_Init(SPI_Handle_t* pSPIHandle) {
	// Error checks
	int8_t handleValidate = SPI_Validate_Handle(pSPIHandle);
	if (handleValidate) return handleValidate;

	// Configure mode
	pSPIHandle->pSPIx->CR[0] &= ~(0x1 << SPI_CR1_MSTR);
	pSPIHandle->pSPIx->CR[0] |= (pSPIHandle->config.mode << SPI_CR1_MSTR);

	// Configure bus
	if (pSPIHandle->config.busConfig == SPI_BUS_FD) {
		pSPIHandle->pSPIx->CR[0] &= ~(0x1 << SPI_CR1_BIDIMODE);
		pSPIHandle->pSPIx->CR[0] &= ~(0x1 << SPI_CR1_RXONLY);
	} else if (pSPIHandle->config.busConfig == SPI_BUS_HD) {
		pSPIHandle->pSPIx->CR[0] |= (0x1 << SPI_CR1_BIDIMODE);
		pSPIHandle->pSPIx->CR[0] &= ~(0x1 << SPI_CR1_RXONLY);
	} else {
		pSPIHandle->pSPIx->CR[0] &= ~(0x1 << SPI_CR1_BIDIMODE);
		pSPIHandle->pSPIx->CR[0] |= (0x1 << SPI_CR1_RXONLY);
	}

	// Configure speed
	pSPIHandle->pSPIx->CR[0] &= ~(0x7 << SPI_CR1_BR);
	pSPIHandle->pSPIx->CR[0] |= (pSPIHandle->config.speed << SPI_CR1_BR);

	// Configure DFF
	pSPIHandle->pSPIx->CR[0] &= ~(0x1 << SPI_CR1_DFF);
	pSPIHandle->pSPIx->CR[0] |= (pSPIHandle->config.DFF << SPI_CR1_DFF);

	// Configure CPOL
	pSPIHandle->pSPIx->CR[0] &= ~(0x1 << SPI_CR1_CPOL);
	pSPIHandle->pSPIx->CR[0] |= (pSPIHandle->config.CPOL << SPI_CR1_CPOL);

	// Configure CPHA
	pSPIHandle->pSPIx->CR[0] &= ~(0x1 << SPI_CR1_CPHA);
	pSPIHandle->pSPIx->CR[0] |= (pSPIHandle->config.CPHA << SPI_CR1_CPHA);

	// Configure SSM
	pSPIHandle->pSPIx->CR[0] &= ~(0x1 << SPI_CR1_SSM);
	pSPIHandle->pSPIx->CR[0] |= (pSPIHandle->config.SSM << SPI_CR1_SSM);

	return 0;
}

/*
 * @fn				SPI_Reset
 *
 * @desc			Resets specified SPI peripheral
 *
 * @param			pSPIx: pointer to peripheral to be reset
 *
 * @return			0			-> success
 * 					NEGATIVE	-> see @ERROR_CODES
 */

int8_t SPI_Reset(SPI_RegDef_t* pSPIx) {
	// Error checks
	if (!pSPIx) return -1;

	switch ((uint32_t) pSPIx) {
		case (uint32_t) SPI1:
			SPI1_REG_RESET();
			break;
		case (uint32_t) SPI2:
			SPI2_REG_RESET();
			break;
		case (uint32_t) SPI3:
			SPI3_REG_RESET();
			break;
		default:
			return -2;
	}

	return 0;
}

/*
 * @fn				SPI_Send
 *
 * @desc			Send specified length of data from TX buffer (blocking call)
 *
 * @param			pSPIHandle: handle of SPI peripheral
 * 					pTxBuffer: pointer to data to be sent
 * 					len: length of data to be sent in bytes
 *
 * @return			0			-> success
 * 					NEGATIVE	-> see @ERROR_CODES
 */

int8_t SPI_Send(SPI_Handle_t* pSPIHandle, uint8_t* pTxBuffer, uint32_t len) {
	// Error checks
	int8_t handleValidate = SPI_Validate_Handle(pSPIHandle);
	if (handleValidate) return handleValidate;
	if (!pTxBuffer) return -13;
	if (len <= 0) return -14;
	if (pSPIHandle->config.DFF == SPI_DFF_16BIT && len % 2 != 0) return -15;

	// Send data from TX Buffer
	while (len > 0) {
		// Wait until TX buffer is empty (flag check)
		while (!(pSPIHandle->pSPIx->SR & (0x1 << SPI_SR_TXE)));

		// Write next data frame into TX buffer
		if (pSPIHandle->config.DFF == SPI_DFF_8BIT) {
			pSPIHandle->pSPIx->DR = *pTxBuffer;

			len--;
			pTxBuffer++;
		} else {
			pSPIHandle->pSPIx->DR = *((uint16_t*) pTxBuffer) ;

			len -= 2;
			pTxBuffer += 2;
		}
	}

	return 0;
}

/*
 * @fn
 *
 * @desc
 *
 * @param
 *
 * @return			0			-> valid handle
 * 					NEGATIVE	-> see @ERROR_CODES
 */

int8_t SPI_Receive(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len);

/*
 * @fn
 *
 * @desc
 *
 * @param
 *
 * @return			0			-> valid handle
 * 					NEGATIVE	-> see @ERROR_CODES
 */

int8_t SPI_IRQEnable(uint8_t IRQNumber, uint8_t enable);

/*
 * @fn
 *
 * @desc
 *
 * @param
 *
 * @return			0			-> valid handle
 * 					NEGATIVE	-> see @ERROR_CODES
 */

int8_t SPI_IRQPriority(uint8_t IRQNumber, uint8_t priority);

/*
 * @fn
 *
 * @desc
 *
 * @param
 *
 * @return			0			-> valid handle
 * 					NEGATIVE	-> see @ERROR_CODES
 */

int8_t SPI_IRQClearPending(SPI_Handle_t* pSPIHandle);
