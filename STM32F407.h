/*
 * STM32F04.h
 *
 *  Created on: 15 feb. 2023
 *      Author: Cesar
 */

#ifndef INC_STM32F407_H_
#define INC_STM32F407_H_

#include <stdint.h>

/*
 * Generic macro definitions
 *
 *
 * */

#define LOW 		0
#define HIGH 		1
#define ENABLE 		HIGH
#define DISABLE 	LOW

#define __vo		volatile

/*
 * Macro definitions for addressable memory locations
 *
 * RAM, ROM, FLASH
 *
 * */

#define SRAM1_BASE_ADDR							0x20000000U
#define SRAM2_BASE_ADDR							0x2001C000U
#define ROM_BASE_ADDR							0x1FFF0000U
#define FLASH_BASE_ADDR							0x08000000U
#define SRAM_BASE_ADDR							SRAM1_BASE_ADDR

/*
 * Macro definitions for addressable memory locations
 *
 * Bus Interfaces
 *
 *
 * */

#define APB1_BASE_ADDR							0x40000000U
#define APB2_BASE_ADDR							0x40010000U
#define AHB1_BASE_ADDR							0x40020000U
#define AHB2_BASE_ADDR							0x50000000U
#define AHB3_BASE_ADDR							0x60000000U
#define PERIPH_BASE								APB1_BASE_ADDR


/*
 * Macro definitions for addressable memory locations
 *
 * APB1 PERIPHERALS
 *
 *
 * */

#define TIM2_BASE								PERIPH_BASE
#define TIM3_BASE								(APB1_BASE_ADDR + 0x0400)
#define TIM4_BASE								(APB1_BASE_ADDR + 0x0800)
#define TIM5_BASE								(APB1_BASE_ADDR + 0x0C00)
#define TIM6_BASE								(APB1_BASE_ADDR + 0x0100)
#define TIM7_BASE								(APB1_BASE_ADDR + 0x1400)
#define TIM12_BASE								(APB1_BASE_ADDR + 0x1800)
#define TIM13_BASE								(APB1_BASE_ADDR + 0x1C00)
#define TIM14_BASE								(APB1_BASE_ADDR + 0x2000)


#define RTC_BKP_BASE							(APB1_BASE_ADDR + 0x2800)
#define WWDG_BASE								(APB1_BASE_ADDR + 0x2800)
#define IWDG_BASE								(APB1_BASE_ADDR + 0x3000)
#define I2S2EXT_BASE							(APB1_BASE_ADDR + 0x3400)
#define SPI2_I2S2_BASE							(APB1_BASE_ADDR + 0x3800)
#define SPI3_I2S3_BASE							(APB1_BASE_ADDR + 0x3C00)
#define I2S3EXT_BASE							(APB1_BASE_ADDR + 0x4000)
#define USART2_BASE								(APB1_BASE_ADDR + 0x4400)
#define USART3_BASE								(APB1_BASE_ADDR + 0x4800)
#define UART4_BASE								(APB1_BASE_ADDR + 0x4C00)
#define UART5_BASE								(APB1_BASE_ADDR + 0x5000)
#define I2C1_BASE								(APB1_BASE_ADDR + 0x5400)
#define I2C2_BASE								(APB1_BASE_ADDR + 0x5800)
#define I2C3_BASE								(APB1_BASE_ADDR + 0x5C00)
#define CAN1_BASE								(APB1_BASE_ADDR + 0x6400)
#define CAN2_BASE								(APB1_BASE_ADDR + 0x6800)
#define PWR_BASE								(APB1_BASE_ADDR + 0x7000)
#define DAC_BASE								(APB1_BASE_ADDR + 0x7400)
#define UART7_BASE								(APB1_BASE_ADDR + 0x7800)
#define UART8_BASE								(APB1_BASE_ADDR + 0x7C00)



/*
 * Macro definitions for addressable memory locations
 *
 * APB2 PERIPHERALS
 *
 *
 * */

#define TIM1_BASE								(APB2_BASE_ADDR + 0x0000)
#define TIM8_BASE								(APB2_BASE_ADDR + 0x0400)
#define USART1_BASE								(APB2_BASE_ADDR + 0x1000)
#define USART6_BASE								(APB2_BASE_ADDR + 0x1400)
#define ADC1_2_3_BASE							(APB2_BASE_ADDR + 0x2000)
#define SDIO_BASE								(APB2_BASE_ADDR + 0x2C00)
#define SPI1_BASE								(APB2_BASE_ADDR + 0x3000)
#define SPI4_BASE								(APB2_BASE_ADDR + 0x3400)

#define SYSCFG_BASE								(APB2_BASE_ADDR + 0x3800)
#define EXTI_BASE								(APB2_BASE_ADDR + 0x3C00)
#define TIM9_BASE								(APB2_BASE_ADDR + 0x4000)
#define TIM10_BASE								(APB2_BASE_ADDR + 0x4400)
#define TIM11_BASE								(APB2_BASE_ADDR + 0x4800)
#define SPI5_BASE								(APB2_BASE_ADDR + 0x5000)
#define SPI6_BASE								(APB2_BASE_ADDR + 0x5400)
#define SAI1_BASE								(APB2_BASE_ADDR + 0x5800)
#define SAI1_BASE								(APB2_BASE_ADDR + 0x5800)
#define LCD_TFT_BASE							(APB2_BASE_ADDR + 0x6800)


/*
 * Macro definitions for addressable memory locations
 *
 * AHB1 PERIPHERALS
 *
 *
 * */

#define GPIOA_BASE								(AHB1_BASE_ADDR + 0x0000)
#define GPIOB_BASE								(AHB1_BASE_ADDR + 0x0400)
#define GPIOC_BASE								(AHB1_BASE_ADDR + 0x0800)
#define GPIOD_BASE								(AHB1_BASE_ADDR + 0x0C00)
#define GPIOE_BASE								(AHB1_BASE_ADDR + 0x1000)
#define GPIOF_BASE								(AHB1_BASE_ADDR + 0x1400)
#define GPIOG_BASE								(AHB1_BASE_ADDR + 0x1800)
#define GPIOH_BASE								(AHB1_BASE_ADDR + 0x1C00)
#define GPIOI_BASE								(AHB1_BASE_ADDR + 0x2000)
#define GPIOJ_BASE								(AHB1_BASE_ADDR + 0x2400)
#define GPIOK_BASE								(AHB1_BASE_ADDR + 0x2800)

#define CRC_BASE								(AHB1_BASE_ADDR + 0x3000)
#define RCC_BASE								(AHB1_BASE_ADDR + 0x3800)
#define BKPSRAM_BASE							(AHB1_BASE_ADDR + 0x4000)
#define DMA1_BASE								(AHB1_BASE_ADDR + 0x6000)
#define DMA2_BASE								(AHB1_BASE_ADDR + 0x6400)
#define ETHERNET_BASE							(AHB1_BASE_ADDR + 0x8000)
#define DMA2D_BASE								(AHB1_BASE_ADDR + 0xB000)
#define USB_OTG_HS_BASE							(AHB1_BASE_ADDR + 0x40000)


/*
 * Macro definitions for addressable memory locations
 *
 * AHB2 PERIPHERALS
 *
 *
 * */

#define USB_OTG_FS_BASE							(AHB2_BASE_ADDR + 0x0000)
#define DCMI_BASE								(AHB2_BASE_ADDR + 0x50000)
#define CRYP_BASE								(AHB2_BASE_ADDR + 0x60000)
#define HASH_BASE								(AHB2_BASE_ADDR + 0x60400)
#define RNG_BASE								(AHB2_BASE_ADDR + 0x60800)


////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 *
 *
 *
 * 		Structs for peripheral registers
 *
 *
 *
 *
 * */


/*
 *
 * GPIO Registers
 *
 * */

typedef struct
{
	__vo uint32_t MODER;				//GPIO port mode register
	__vo uint32_t OTYPER;				//GPIO port output type register
	__vo uint32_t OSPEEDR;				//GPIO port output speed register
	__vo uint32_t PUPDR;				//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;					//GPIO port input data register
	__vo uint32_t ODR;					//GPIO port output data register
	__vo uint32_t BSRR;					//GPIO port bit set/reset register
	__vo uint32_t LCKR;					//GPIO port configuration lock register
	__vo uint32_t AFRL;					//GPIO alternate function low register
	__vo uint32_t AFRH;					//GPIO alternate function high register

}GPIO_RegDef_t;


/*
 *
 * RCC Registers
 *
 * */

typedef struct
{
	__vo uint32_t CR;					//RCC clock control register
	__vo uint32_t PLLCFGR;				//RCC PLL configuration register
	__vo uint32_t CFGR;					//RCC clock configuration register
	__vo uint32_t CIR;					//RCC clock interrupt register
	__vo uint32_t AHB1RSTR;				//RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;				//RCC AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR;				//RCC AHB3 peripheral reset register
	__vo uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;				//RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR;				//RCC APB2 peripheral reset register
	__vo uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;				//RCC AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;				//RCC AHB2 peripheral clock enable register
	__vo uint32_t AHB3ENR;				//RCC AHB3 peripheral clock enable register
	__vo uint32_t RESERVED2;
	__vo uint32_t APB1ENR;				//RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR;				//RCC APB2 peripheral clock enable register
	__vo uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;			//RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR;			//RCC AHB2 peripheral clock enable in low power mode register
	__vo uint32_t AHB3LPENR;			//RCC AHB3 peripheral clock enable in low power mode register
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;			//RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LPENR;			//RCC APB2 peripheral clock enable in low power mode register
	__vo uint32_t RESERVED5[2];
	__vo uint32_t BDCR;					//RCC Backup domain control register
	__vo uint32_t CSR;					//RCC clock control & status register
	__vo uint32_t SSCGR;				//RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;			//RCC PLLI2S configuration register


}RCC_RegDef_t;


/*
 *
 * I2C Registers
 *
 * */

typedef struct
{

	__vo uint32_t CR1;					//I2C Control register 1
	__vo uint32_t CR2;					//I2C Control register 2
	__vo uint32_t OAR1;					//I2C Own address register 1
	__vo uint32_t OAR2;					//I2C Own address register 1
	__vo uint32_t DR;					//I2C Data register
	__vo uint32_t SR1;					//I2C Status register 1
	__vo uint32_t SR2;					//I2C Status register 2
	__vo uint32_t CCR;					//I2C Clock control register
	__vo uint32_t TRISE;				//I2C TRISE register
	__vo uint32_t FLTR;					//I2C FLTR register

} I2C_RegDef_t;


/*
 *
 * SPI Registers
 *
 * */


typedef struct
{

	__vo uint32_t CR1;					//SPI control register 1
	__vo uint32_t CR2;					//SPI control register 2
	__vo uint32_t SR;					//SPI status register
	__vo uint32_t DR;					//SPI data register
	__vo uint32_t CRCPR;				//SPI CRC polynomial register
	__vo uint32_t RXCRCR;				//SPI RX CRC register
	__vo uint32_t TXCRCR;				//SPI TX CRC register
	__vo uint32_t I2SCFGR;				//SPI_I2S configuration register
	__vo uint32_t I2SPR;				//SPI_I2S prescaler register


} SPI_RegDef_t;


/*
 *
 * SYSCFG Registers
 *
 * */

typedef struct
{

	__vo uint32_t MEMRMP;				//SYSCFG memory remap register
	__vo uint32_t PMC;					//SYSCFG peripheral mode configuration register
	__vo uint32_t EXTICR1;				//SYSCFG external interrupt configuration register 1
	__vo uint32_t EXTICR2;				//SYSCFG external interrupt configuration register 2
	__vo uint32_t EXTICR3;				//SYSCFG external interrupt configuration register 3
	__vo uint32_t EXTICR4;				//SYSCFG external interrupt configuration register 4
	__vo uint32_t CMPCR;				//Compensation cell control register


} SYSCFG_RegDef_t;


/*
 *
 * USART Registers
 *
 * */

typedef struct
{
	__vo uint32_t SR;					//Status register
	__vo uint32_t DR;					//Data register
	__vo uint32_t BRR;					//Baud rate register
	__vo uint32_t CR1;					//Control register 1
	__vo uint32_t CR2;					//Control register 2
	__vo uint32_t CR3;					//Control register 3
	__vo uint32_t GTPR;					//Guard time and prescaler register

} USART_RegDef_t;



typedef struct
{

	__vo uint32_t IMR;				//Interrupt mask register
	__vo uint32_t EMR;				//Event mask register
	__vo uint32_t RTSR;				//Rising trigger selection reg
	__vo uint32_t FTSR;				//Falling trigger selection reg
	__vo uint32_t SWIER; 			//SW Interrupt event register
	__vo uint32_t PR;				//Pending register

}EXTI_RegDef_t;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 *
 *
 *
 * 		Dirección de periféricos convertida al tipo de puntero a trabajar
 *
 *
 *
 *
 * */


#define		RCC				((RCC_RegDef_t*)RCC_BASE)	//RCC is a pointer to the RCC Register Structure
#define		EXTI			((EXTI_RegDef_t*)EXTI_BASE)

#define 	GPIOA			((GPIO_RegDef_t*)GPIOA_BASE)//GPIOA is a pointer to the GPIOA Register Structure
#define 	GPIOB			((GPIO_RegDef_t*)GPIOB_BASE)
#define 	GPIOC			((GPIO_RegDef_t*)GPIOC_BASE)
#define 	GPIOD			((GPIO_RegDef_t*)GPIOD_BASE)
#define 	GPIOE			((GPIO_RegDef_t*)GPIOE_BASE)
#define 	GPIOF			((GPIO_RegDef_t*)GPIOF_BASE)
#define 	GPIOG			((GPIO_RegDef_t*)GPIOG_BASE)
#define 	GPIOH			((GPIO_RegDef_t*)GPIOH_BASE)
#define 	GPIOI			((GPIO_RegDef_t*)GPIOI_BASE)
#define 	GPIOJ			((GPIO_RegDef_t*)GPIOJ_BASE)
#define 	GPIOK			((GPIO_RegDef_t*)GPIOK_BASE)


#define 	I2C1			((I2C_RegDef_t*)I2C1_BASE) //I2C is a pointer to the I2C Register Structure
#define 	I2C2			((I2C_RegDef_t*)I2C2_BASE)
#define 	I2C3			((I2C_RegDef_t*)I2C3_BASE)


#define		SPI1			((SPI_RegDef_t*)SPI1_BASE)	//SPI1 is a pointer to the I2C Register Structure
#define		SPI2			((SPI_RegDef_t*)SPI2_I2S2_BASE)
#define		SPI3			((SPI_RegDef_t*)SPI3_I2S3_BASE)
#define		SPI4			((SPI_RegDef_t*)SPI4_BASE)


////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 *
 *
 *
 * 		Habilitación/Deshabilitación de relojes para los diferentes periféricos
 *
 *
 *
 *
 * */


/*
 *
 * GPIO Clock enable macros
 *
 * */

#define			GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0))
#define			GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define			GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define			GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define			GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))
#define			GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1 << 5))
#define			GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1 << 6))
#define			GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7))
#define			GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1 << 8))


/*
*
* I2C Clock enable macros
*
* */

#define			I2C1_PCLK_EN()			(RCC->APB1ENR |= (1 << 21))
#define			I2C2_PCLK_EN()			(RCC->APB1ENR |= (1 << 22))
#define			I2C3_PCLK_EN()			(RCC->APB1ENR |= (1 << 23))


/*
*
* SPI Clock enable macros
*
* */

#define			SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12))
#define			SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14))
#define			SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15))
#define			SPI4_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 *
 * GPIO Clock DISABLE macros
 *
 * */

#define			GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 0))
#define			GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 1))
#define			GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 2))
#define			GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 3))
#define			GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 4))
#define			GPIOF_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 5))
#define			GPIOG_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 6))
#define			GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 7))
#define			GPIOI_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 8))


/*
*
* I2C Clock DISABLE macros
*
* */

#define			I2C1_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 21))
#define			I2C2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 22))
#define			I2C3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 23))


/*
*
* SPI Clock DISABLE macros
*
* */

#define			SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12))
#define			SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 14))
#define			SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 15))
#define			SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 *
 * RCC AHB1 Reset macros
 *
 * */

#define			GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define			GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define			GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define			GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define			GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define			GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define			GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define			GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)
#define			GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));} while(0)


#include "STM32F407_GPIO_Drivers.h"


#endif /* INC_STM32F407_H_*/
