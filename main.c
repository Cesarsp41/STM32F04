/*
 * main.c
 *
 *  Created on: 22 feb. 2023
 *      Author: Cesar
 */
#include <stdio.h>
#include "STM32F407.h" //Includes MCU Specific header and GPIO API

int main (void)
{
	GPIO_Handle_t Led;		//Creates a struct for handling LED

	Led.pGPIOx = GPIOD;		//Initialices GPIOD Base Address
	Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;		//Output mode
	Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_PP;		//Push pull output
	Led.GPIO_PinConfig.GPIO_PinNumber = 12;					//Pin 12
	Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;	//High speed
	Led.GPIO_PinConfig.GPIO_PuPdConfig = GPIO_PUPD_NO;		//No push pull

	GPIOD_PCLK_EN();										//Enable PortD Clock
	GPIO_Init(&Led);

	getchar();
	return 0;
}
