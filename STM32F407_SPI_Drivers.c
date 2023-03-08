/*
 * STM32F407_SPI_Drivers.c
 *
 *  Created on: 7 mar. 2023
 *      Author: Cesar
 */

#include "STM32F407_SPI_Drivers.h"

/*
 *
 * Function Definitions
 *
 * */


/*************************************************************
 *
 * @Function					SPI_PCLKCTRL
 *
 * @Description					This function activates or deactivates
 * 								SPI Clock
 *
 * @param[pSPIx]				Stores base address of actual SPIx Peripheral
 *
 * @param[EN_DI]				Macro ENABLE or DISABLE. Defined in MCU header
 *
 * @return						void
 *
 * @Note						none
 *
 * */

void SPI_PCLKCTRL (SPI_RegDef_t *pSPIx, uint8_t EN_DI)
{
	if (EN_DI == ENABLE)
	{
		if (pSPIx == SPI1) SPI1_PCLK_EN();
		if (pSPIx == SPI2) SPI2_PCLK_EN();
		if (pSPIx == SPI3) SPI3_PCLK_EN();
		if (pSPIx == SPI4) SPI4_PCLK_EN();
	}
	else
	{
		if (pSPIx == SPI1) SPI1_PCLK_DI();
		if (pSPIx == SPI2) SPI2_PCLK_DI();
		if (pSPIx == SPI3) SPI3_PCLK_DI();
		if (pSPIx == SPI4) SPI4_PCLK_DI();
	}
}

/*************************************************************
 *
 * @Function					SPI_Init
 *
 * @Description					This function uses SPI_Config_t structure data
 * 								to initialize SPI Registers
 *
 * @param[pSPIHandle]			Pointer to Handle structure to access Config_t
 *
 *
 * @return						void
 *
 * @Note						none
 *
 * */

void SPI_Init (SPI_Handle_t *pSPIHandle)
{

	uint32_t temp = 0;
	/*
	 * 1- Device mode: Master / Slave
	 *
	 * This is done through SPI Control Register 1, activating
	 * MSTR bit
	 *
	 * MSTR = 0 -> Slave Configuration
	 * MSTR = 1 -> Master Configuration
	 *
	 * */

	temp |= (pSPIHandle->SPIConfig.SPI_DeviceMode << 2);





	/*
	 * 2- Bus Configuration: Full-Duplex, Half-Duplex, Simplex
	 *
	 * This is done through SPI Control Register 1, activating
	 * BIDIMODE
	 *
	 *
	 * BIDIMODE = 0 -> 2 Line UNIDIRECTIONAL data mode (Full-Duplex)
	 * BIDIMODE = 1 -> 1-Line BIDIRECTIONAL data mode (Half-Duplex)
	 *
	 * En el caso de ser Half-Duplex, debe decidirse quién es el
	 * Transmisor (TX) y quién el receptor (RX). Para esto, está el bit
	 * BIDIOE
	 *
	 * BIDIOE = 0 -> RX Mode
	 * BIDIOE = 1 -> TX Mode
	 *
	 * En el caso de necesitar Simplex:
	 *
	 * BIDIMODE = 0 (Unidireccional)
	 *
	 * Y adicionalmente:
	 *
	 * RXONLY = 1 -> Receive ONLY
	 *
	 * Y en el caso de necesitar Simplex, transmiting only:
	 * Usar full-duplex y solo ignorar MISO
	 *
	 * */

	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_FD)
	{
		temp &= ~(1 << 15);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_HD)
	{
		temp |= (1 << 15);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_SIMPLEX_RXONLY)
	{
		temp &= ~(1 << 15);
		temp |= (1 << 10);
	}


	/*
	 * 3- DFF: 8 or 16 bits
	 *
	 * This is done through SPI Control Register 1
	 *
	 * DFF = 0 -> 8-bit
	 * DFF = 1 -> 16-bit
	 *
	 * */



	/*
	 * 4- CPHA
	 *
	 * Done through SPI Control Register 1
	 *
	 * CPHA = 0 Datos capturados en subida, datos leidos en bajada
	 * CPHA = 1 Datos capturados en bajada, datos leidos en subida
	 *
	 * */


	/*
	 * 5- CPOL
	 *
	 * Done though SPI Control Register 1
	 *
	 * CPOL = 0 -> CLK 0 en inactivo
	 * CPOL = 1 -> CLK 1 en inactivo
	 *
	 * */

	/*
	 * 6- SSM
	 *
	 * Done though SPI Control Register 1
	 *
	 * SSM = 0 -> SW Slave Management disabled
	 * SSM = 1 -> SW slave management enabled
	 *
	 * */




}

/*************************************************************
 *
 * @Function
 *
 * @Description
 *
 * @param[]
 *
 * @param[]
 *
 * @return
 *
 * @Note
 *
 * */

void SPI_DeInit (SPI_RegDef_t *pSPIx)
{

}

/*************************************************************
 *
 * @Function
 *
 * @Description
 *
 * @param[]
 *
 * @param[]
 *
 * @return
 *
 * @Note
 *
 * */

void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t lenght)
{

}

/*************************************************************
 *
 * @Function
 *
 * @Description
 *
 * @param[]
 *
 * @param[]
 *
 * @return
 *
 * @Note
 *
 * */

void SPI_ReceiveData (SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t lenght)
{

}

/*************************************************************
 *
 * @Function
 *
 * @Description
 *
 * @param[]
 *
 * @param[]
 *
 * @return
 *
 * @Note
 *
 * */

void SPI_IRQConfig (uint8_t IRQNumber, uint8_t EN_DI)
{

}

/*************************************************************
 *
 * @Function
 *
 * @Description
 *
 * @param[]
 *
 * @param[]
 *
 * @return
 *
 * @Note
 *
 * */

void SPI_IRQHandle (SPI_Handle_t *pSPIHandle)
{

}

/*************************************************************
 *
 * @Function
 *
 * @Description
 *
 * @param[]
 *
 * @param[]
 *
 * @return
 *
 * @Note
 *
 * */

void SPI_IQRPriorityConfig (uint8_t IRQPriority, uint8_t IRQNumber)
{

}

