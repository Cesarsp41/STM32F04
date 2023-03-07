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

/*
 *
 * 	Macro definitions for SPI
 *
 * */







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



#endif /* INC_STM32F407_SPI_DRIVERS_H_ */
