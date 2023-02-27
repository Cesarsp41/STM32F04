/*
 * STM32F407_GPIO_Drivers.c
 *
 *  Created on: 17 feb. 2023
 *      Author: Cesar
 */


#include "STM32F407_GPIO_Drivers.h"

/*
 *
 * 		Function Definition
 *
 *
 * */




/*************************************************************
 *
 * @Function					GPIO_PCLKCTRL
 *
 * @Description					Enables or disables the given GPIO
 * 								peripheral
 * 								clock through RCC AHB1ENR Register
 *
 * @param[pGPIOx]				Base Address of GPIO Port
 *
 * @param[EN_DI]				ENABLE | DISABLE Macro
 *
 * @return						void
 *
 * @Note						none
 *
 * */


void GPIO_PCLKCTRL (GPIO_RegDef_t *pGPIOx, uint8_t EN_DI)
{
	if (EN_DI == ENABLE)
	{
		if (pGPIOx == GPIOA) GPIOA_PCLK_EN();
		if (pGPIOx == GPIOB) GPIOB_PCLK_EN();
		if (pGPIOx == GPIOC) GPIOC_PCLK_EN();
		if (pGPIOx == GPIOD) GPIOD_PCLK_EN();
		if (pGPIOx == GPIOE) GPIOE_PCLK_EN();
		if (pGPIOx == GPIOF) GPIOF_PCLK_EN();
		if (pGPIOx == GPIOG) GPIOG_PCLK_EN();
		if (pGPIOx == GPIOH) GPIOH_PCLK_EN();
		if (pGPIOx == GPIOI) GPIOI_PCLK_EN();

	}
	else
	{
		if (pGPIOx == GPIOA) GPIOA_PCLK_DI();
		if (pGPIOx == GPIOB) GPIOB_PCLK_DI();
		if (pGPIOx == GPIOC) GPIOC_PCLK_DI();
		if (pGPIOx == GPIOD) GPIOD_PCLK_DI();
		if (pGPIOx == GPIOE) GPIOE_PCLK_DI();
		if (pGPIOx == GPIOF) GPIOF_PCLK_DI();
		if (pGPIOx == GPIOG) GPIOG_PCLK_DI();
		if (pGPIOx == GPIOH) GPIOH_PCLK_DI();
		if (pGPIOx == GPIOI) GPIOI_PCLK_DI();
	}
}


/*************************************************************
 *
 * @Function					GPIO_Init
 *
 * @Description					Initializes GPIO Registers with GPIO Configuration structure information
 *
 * @param[*pGPIOHandle]			Pointer to Handle structure
 *
 *
 *
 * @return						void
 *
 * @Note						none
 *
 * */


void GPIO_Init (GPIO_Handle_t *pGPIOHandle)
{
	__vo uint32_t temp; //Temp Register

	// 1 - Configure Pin Mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Shift Pin mode into TEMP
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//Clear reg Bits
		pGPIOHandle->pGPIOx->MODER |= temp; //Mask MODER to TEMP
		temp = 0;	//Clean Temp
	}
	else
	{
		//Interrupt modes configuration

		//Rising / Falling edge config
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)		//Rising Edge Trigger
		{
			//Limpiar FT primero y luego set RT
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)		//Falling Edge Trigger
		{
			//Limpiar RT y luego set FT
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else 			//Rising AND Falling Edge Trigger
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

	}



	// 2 - Configure Output Type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Shift OP Type into TEMP
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//Clear reg Bits
	pGPIOHandle->pGPIOx->OTYPER |= temp; //Mask MODER to TEMP
	temp = 0;	//Clean Temp



	// 3 - Configure Output Speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Shift OP Speed into TEMP
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//Clear reg Bits
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; //Mask MODER to TEMP
	temp = 0;	//Clean Temp



	// 4 - Configure PullUp - PullDown
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PuPdConfig << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Shift PUPD into TEMP
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//Clear reg Bits
	pGPIOHandle->pGPIOx->PUPDR |= temp; //Mask MODER to TEMP
	temp = 0;	//Clean Temp



	// 5 - Configure Alt Function
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7) //Low Register
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFun << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->AFRL &= ~(0x0F << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->AFRL |= temp;
		temp = 0;
	}
	else // High Register
	{
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFun << (4 * temp2));
		pGPIOHandle->pGPIOx->AFRH &= ~(0x0F << temp2);
		pGPIOHandle->pGPIOx->AFRH |= temp;
		temp = 0;
	}

}


/*************************************************************
 *
 * @Function					GPIO_DeInit
 *
 * @Description					Resets whole GPIO Port. This is done through AHB1RSTR RCC Register
 * 								(AHB1 Reset Register)
 *
 * @param[*pGPIOx]		   		GPIO Port Base's address
 *
 *
 *
 * @return						void
 *
 * @Note						none
 *
 * */


void GPIO_DeInit (GPIO_RegDef_t *pGPIOx)
{

	if (pGPIOx == GPIOA) GPIOA_REG_RESET();
	if (pGPIOx == GPIOB) GPIOB_REG_RESET();
	if (pGPIOx == GPIOC) GPIOC_REG_RESET();
	if (pGPIOx == GPIOD) GPIOD_REG_RESET();
	if (pGPIOx == GPIOE) GPIOE_REG_RESET();
	if (pGPIOx == GPIOF) GPIOF_REG_RESET();
	if (pGPIOx == GPIOG) GPIOG_REG_RESET();
	if (pGPIOx == GPIOH) GPIOH_REG_RESET();
	if (pGPIOx == GPIOI) GPIOI_REG_RESET();

}


/*************************************************************
 *
 * @Function					GPIO_WritePin
 *
 * @Description					Digital Output write to a GPIO Pin number
 *
 * @param[*pGPIOx]				GPIO Port Base's address
 *
 * @param[PinNumber]			GPIO Pin Number
 *
 * @param[Value]				HIGH | LOW 	Macro
 *
 *
 * @return						void
 *
 * @Note						none
 *
 * */


void GPIO_WritePin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == HIGH)
		pGPIOx->ODR |= (1 << PinNumber);
	else
		pGPIOx->ODR &= ~(1 << PinNumber);
}


/*************************************************************
 *
 * @Function					GPIO_WritePort
 *
 * @Description					Digital Output write to a GPIO PORT
 *
 * @param[*pGPIOx]				GPIO Port Base's address
 *
 *
 * @param[Value]				16 Bit long data
 *
 *
 * @return						void
 *
 * @Note						none
 *
 * */


void GPIO_WritePort (GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/*************************************************************
 *
 * @Function					GPIO_ReadPin
 *
 * @Description					Digital read of GPIO Pin
 *
 * @param[*pGPIOx]				GPIO Port Base's address
 *
 * @param[PinNumber]			Pin Number
 *
 *
 *
 * @return						Binary data of GPIO Pin stored in uint8_t ( 0 | 1 )
 *
 * @Note						none
 *
 * */


uint8_t GPIO_ReadPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x0000001);
	return value;
}


/*************************************************************
 *
 * @Function					GPIO_ReadPort
 *
 * @Description					Digital read of GPIO PORT
 *
 * @param[*pGPIOx]				GPIO Port Base's address
 *
 *
 *
 *
 * @return						Binary data of GPIO Port stored in uint16_t
 *
 * @Note						none
 *
 * */


uint16_t GPIO_ReadPort (GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}


/*************************************************************
 *
 * @Function					GPIO_OPConfig
 *
 * @Description					Output type from: Open drain, push upll
 *
 * @param[*pGPIOx]				GPIO Port Base's address
 *
 * @param[PinNumber]			Pin Number
 *
 *
 * @return						void
 *
 * @Note						none
 *
 * */


void GPIO_OPConfig (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{


}


/*************************************************************
 *
 * @Function					GPIO_IRQConfig
 *
 * @Description					Handles GPIO Interrupt.
 *
 *
 * @param[IRQNumber]			IRQ Numbmer (From which Pin interrupt comes from?)
 *
 * @param[EN_DI]				ENABLE | DISABLE	Macros

 * @param[IRQPriority]			Interrupt priority
 *
 * @return						void
 *
 * @Note						Note to self:
 * 								-What do I need to hande GPIO Interrupt?
 * 								R - IRQ Number (EXTI0 - EXTI16)
 * 								R - Priority
 *
 * */


void GPIO_IRQConfig (uint8_t IRQNumber, uint8_t EN_DI, uint8_t IRQPriority)
{



}


/*************************************************************
 *
 * @Function					GPIO_IRQHandle
 *
 * @Description					Handles GPIO Interrupt.
 *
 *
 * @param[PinNumber]			Pin Number (From which Pin interrupt comes from?)
 *
 *
 * @return						void
 *
 * @Note						Note to self:
 * 								-What do I need to hande GPIO Interrupt?
 * 								R - IRQ Number (EXTI0 - EXTI16)
 * 								R - Priority
 *
 * */


void GPIO_IRQHandle (uint8_t PinNumber)
{



}
