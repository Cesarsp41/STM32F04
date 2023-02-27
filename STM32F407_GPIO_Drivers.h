/*
 * STM32F407_GPIO_Drivers.h
 *
 *  Created on: 17 feb. 2023
 *      Author: Cesar
 */

#ifndef INC_STM32F407_GPIO_DRIVERS_H_
#define INC_STM32F407_GPIO_DRIVERS_H_
#include "STM32F407.h"
#include <stdint.h>

/************************************************
 *
 *  MACRO DEFINITIONS
 *
 * */

//Mode Macros
#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_ALTFUN 2
#define GPIO_MODE_ANALOG 3

/*
 * Interrupt Modes
 * */
#define GPIO_MODE_IT_RT 4
#define GPIO_MODE_IT_FT 5
#define GPIO_MODE_IT_RFT 6


//OP Speed Macros
#define GPIO_OP_SPEED_LOW 0
#define GPIO_OP_SPEED_MEDIUM 1
#define GPIO_OP_SPEED_HIGH 2
#define GPIO_OP_SPEED_VHIGH 3

//Pull UP Pull Down Config
#define GPIO_PUPD_NO 0
#define GPIO_PUPD_PU 1
#define GPIO_PUPD_PD 2

//OP Type
#define GPIO_OPT_PP 0 //Push Pull
#define GPIO_OPT_OD 1 //Open Drain

//Alt Function
#define GPIO_AF0 0
#define GPIO_AF1 1
#define GPIO_AF2 2
#define GPIO_AF3 3
#define GPIO_AF4 4
#define GPIO_AF5 5
#define GPIO_AF6 6
#define GPIO_AF7 7
#define GPIO_AF8 8
#define GPIO_AF9 9
#define GPIO_AF10 10
#define GPIO_AF11 11
#define GPIO_AF12 12
#define GPIO_AF13 13
#define GPIO_AF14 14
#define GPIO_AF15 15


//Config Structure
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PuPdConfig;
	uint8_t GPIO_PinAltFun;


}GPIO_Config_t;


//Handle Structure
typedef struct
{
	GPIO_RegDef_t *pGPIOx; 		//Port Base Address
	GPIO_Config_t GPIO_PinConfig;		//Configuration structure

}GPIO_Handle_t;



/*
 *
 * 		Function prototypes
 *
 *
 * */

//Set-Up
void GPIO_PCLKCTRL (GPIO_RegDef_t *pGPIOx, uint8_t EN_DI); //Enable or Disable Peripheral Clock


void GPIO_Init (GPIO_Handle_t *pGPIOHandle); //Initialize Handle struct
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx); //Usa el registro RCC periph reset


//Data Read/Write
void GPIO_WritePin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritePort (GPIO_RegDef_t *pGPIOx, uint16_t Value);

uint8_t GPIO_ReadPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber); //Reads from pin
uint16_t GPIO_ReadPort (GPIO_RegDef_t *pGPIOx); //Reads from 16 Long port


//Config output
void GPIO_OPConfig (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber); //Tipo de salida

//Interrupt hadle
void GPIO_IRQConfig (uint8_t IRQNumber, uint8_t EN_DI, uint8_t IRQPriority);
void GPIO_IRQHandle (uint8_t PinNumber);








#endif /* INC_STM32F407_GPIO_DRIVERS_H_ */
