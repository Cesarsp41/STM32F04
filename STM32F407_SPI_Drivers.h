/*
 * STM32F407_SPI_Drivers.h
 *
 *  Created on: 7 mar. 2023
 *      Author: Cesar
 */

#ifndef INC_STM32F407_SPI_DRIVERS_H_
#define INC_STM32F407_SPI_DRIVERS_H_

#include "STM32F407.h"
#include <stdint.h>

/*******************************************************
 *
 * 	Macro definitions for SPI
 *
 * */

/*
 *
 * Device Mode Macros
 *
 * */


#define SPI_DEVICE_MODE_SLAVE 	0
#define SPI_DEVICE_MODE_MASTER	1


/*
 *
 * Bus Configuration Macros
 *
 * */

#define SPI_BUSCONFIG_FD					1
#define SPI_BUSCONFIG_HD					2
#define SPI_BUSCONFIG_SIMPLEX_RXONLY 		3

/*
 *
 * Clock Speed Macros
 *
 * */

#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7


/*
 *
 * DFF Data Frame Format Macros
 *
 * */

#define SPI_DFF_8BITS 			0
#define SPI_DFF_16BITS 			1

/*
 *
 * CPOL Macros
 *
 * */

#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/*
 *
 * CPHA Macros
 *
 * */

#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0


/*
 *
 * SSM Macros
 *
 * */

#define SPI_SSM_EN			1
#define SPI_SSM_DI			0


/*
 *
 *  SPI FLAGS MACROS
 *
 * */

#define SPI_TXE_FLAG 	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG	(1 << SPI_SR_BSY)

/*
 *
 * 		SPI Configuration Structure
 *
 * */

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_DFF;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;
	uint8_t SPI_Speed;

}SPI_Config_t;

/*
 *
 * 		SPI Handle Structure
 *
 * */

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;

}SPI_Handle_t;


/*
 *
 * 		SPI Function prototypes
 *
 * */

/*
 *
 * Set Up	Clock Control and Init
 *
 */
void SPI_PCLKCTRL (SPI_RegDef_t *pSPIx, uint8_t EN_DI); //Enable or Disable Peripheral Clock


void SPI_Init (SPI_Handle_t *pSPIHandle); //Initialize Handle struct
void SPI_DeInit (SPI_RegDef_t *pSPIx); //Usa el registro RCC periph reset



/*
 *
 * Data Send & Receive
 *
 */

void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t lenght);
void SPI_ReceiveData (SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t lenght);


/*
 *
 * IRQ Configuration and ISR Handler
 *
 */

void SPI_IRQConfig (uint8_t IRQNumber, uint8_t EN_DI);
void SPI_IRQHandle (SPI_Handle_t *pSPIHandle);
void SPI_IQRPriorityConfig (uint8_t IRQPriority, uint8_t IRQNumber);









#endif /* INC_STM32F407_SPI_DRIVERS_H_ */
