//Header file for STM32F04 MCU 
#include <stdint.h>
//Define RAM, Flash, ROM, Base addresses RCC, GPIOS, I2C, CAN, SPI, USART

//Macro definitions for FLASH, ROM, RAM
#define FLASH__BASE_ADDR                0x08000000U
#define ROM_BASE_ADDR                   0x1FFF0000U
#define SRAM1_BASE_ADDR                 0x20000000U
#define SRAM2_BASE                      0x2001C000U
#define RAM_BASE_ADDR                   SRAM1_BASE_ADDR



//Macro definitions for Bus base addresses
#define AHB1_BASE_ADDR                  0x40020000U
#define AHB2_BASE_ADDR                  0x50000000U
#define AHB3_BASE_ADDR                  0x60000000U
#define APB1_BASE_ADDR                  0x40000000U
#define APB2_BASE_ADDR                  0x40010000U



//Macro definitions for AHB1 peripherals
#define GPIOA_BASE_ADDR                 (AHB1_BASE_ADDR + 0x0000) 
#define GPIOB_BASE_ADDR                 (AHB1_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR                 (AHB1_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR                 (AHB1_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR                 (AHB1_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR                 (AHB1_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR                 (AHB1_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR                 (AHB1_BASE_ADDR + 0x1C00)
#define GPIOI_BASE_ADDR                 (AHB1_BASE_ADDR + 0x2000)
#define GPIOJ_BASE_ADDR                 (AHB1_BASE_ADDR + 0x2400)
#define GPIOK_BASE_ADDR                 (AHB1_BASE_ADDR + 0x2800) //Done GPIO register structure
#define CRC_BASE_ADDR                   (AHB1_BASE_ADDR + 0x3000) //Done CRC register structure
#define RCC_BASE_ADDR                   (AHB1_BASE_ADDR + 0x3800) //Done RCC register structure



//Macro definitions for APB1 peripherals
#define TIM2_BASE_ADDR                  (APB1_BASE_ADDR + 0x0000) //General-purpose timers (TIM2 to TIM5)
#define TIM3_BASE_ADDR                  (APB1_BASE_ADDR + 0x0400) 
#define TIM4_BASE_ADDR                  (APB1_BASE_ADDR + 0x0800) 
#define TIM5_BASE_ADDR                  (APB1_BASE_ADDR + 0x0C00) //Done GPT structure


#define TIM6_BASE_ADDR                  (APB1_BASE_ADDR + 0x1000)
#define TIM7_BASE_ADDR                  (APB1_BASE_ADDR + 0x1400) /* ToDo */


#define TIM12_BASE_ADDR                 (APB1_BASE_ADDR + 0x1800)
#define TIM13_BASE_ADDR                 (APB1_BASE_ADDR + 0x1C00)
#define TIM14_BASE_ADDR                 (APB1_BASE_ADDR + 0x2000) /* ToDo */


#define RTC_BKP_REG_BASE_ADDR           (APB1_BASE_ADDR + 0x2800)
#define WWDG_BASE_ADDR                  (APB1_BASE_ADDR + 0x2C00)
#define IWDG_BASE_ADDR                  (APB1_BASE_ADDR + 0x3000)
#define I2S2ext_BASE_ADDR               (APB1_BASE_ADDR + 0x3400)
#define SPI2_BASE_ADDR                  (APB1_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR                  (APB1_BASE_ADDR + 0x3C00)
#define I2S3ext_BASE_ADDR               (APB1_BASE_ADDR + 0x4000)
#define USART2_BASE_ADDR                (APB1_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR                (APB1_BASE_ADDR + 0x4800)
#define UART4_BASE_ADDR                 (APB1_BASE_ADDR + 0x4C00)
#define UART5_BASE_ADDR                 (APB1_BASE_ADDR + 0x5000) /* ToDo */


#define I2C1_BASE_ADDR                  (APB1_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR                  (APB1_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR                  (APB1_BASE_ADDR + 0x5C00) //Done I2C Struct register


#define CAN1_BASE_ADDR                  (APB1_BASE_ADDR + 0x6400)
#define CAN2_BASE_ADDR                  (APB1_BASE_ADDR + 0x6800) /* ToDo*/


#define PWR_BASE_ADDR                   (APB1_BASE_ADDR + 0x7000)
#define DAC_BASE_ADDR                   (APB1_BASE_ADDR + 0x7400)
#define UART7_BASE_ADDR                 (APB1_BASE_ADDR + 0x7800)
#define UART8_BASE_ADDR                 (APB1_BASE_ADDR + 0x7C00) /* ToDo */


//Macro definitions for APB2 peripherals
#define TIM1_BASE_ADDR                  (APB2_BASE_ADDR + 0x0000)
#define TIM8_BASE_ADDR                  (APB2_BASE_ADDR + 0x0400)
#define USART1_BASE_ADDR                (APB2_BASE_ADDR + 0x1000)
#define USART6_BASE_ADDR                (APB2_BASE_ADDR + 0x1400)
#define ADC1_BASE_ADDR                  (APB2_BASE_ADDR + 0x2000)
#define SDIO_BASE_ADDR                  (APB2_BASE_ADDR + 0x2C00)
#define SPI1_BASE_ADDR                  (APB2_BASE_ADDR + 0x3000)
#define SPI4_BASE_ADDR                  (APB2_BASE_ADDR + 0x3400)
#define SYSCFG_BASE_ADDR                (APB2_BASE_ADDR + 0x3800)
#define EXTI_BASE_ADDR                  (APB2_BASE_ADDR + 0x3C00)
#define TIM9_BASE_ADDR                  (APB2_BASE_ADDR + 0x4000)
#define TIM10_BASE_ADDR                 (APB2_BASE_ADDR + 0x4400)
#define TIM11_BASE_ADDR                 (APB2_BASE_ADDR + 0x4800)
#define SPI5_BASE_ADDR                  (APB2_BASE_ADDR + 0x5000)
#define SPI6_BASE_ADDR                  (APB2_BASE_ADDR + 0x5400)
#define SAI1_BASE_ADDR                  (APB2_BASE_ADDR + 0x5800)
#define LCD_BASE_ADDR                   (APB2_BASE_ADDR + 0x6800) /* ToDo */



// structure for GPIO Registers
typedef struct
{
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRL;
    volatile uint32_t AFRH;

}GPIO_RegDef_t;


//GPIO base addresses to use with GPIO_RegDef structure pointers (typecasted)
#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI ((GPIO_RegDef_t*)GPIOI_BASE_ADDR)
#define GPIOJ ((GPIO_RegDef_t*)GPIOJ_BASE_ADDR)
#define GPIOK ((GPIO_RegDef_t*)GPIOK_BASE_ADDR)



// structure for CRC registers
typedef struct
{
    volatile uint32_t DR;                       //Data register
    volatile uint32_t IDR;                      //Independent data register
    volatile uint32_t CR;                       //Control Register

}CRC_RegDef_t;


//structure for RCC registers
typedef struct 
{
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    volatile uint32_t RESERVED0;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    volatile uint32_t RESERVED2;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t RESERVED3[2];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    volatile uint32_t AHB3LPENR;
    volatile uint32_t RESERVED4;
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    volatile uint32_t RESERVED5[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    volatile uint32_t RESERVED6[2];
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;

}RCC_RegDef_t;


//RCC base addresses to use with RCC_RegDef structure pointers (typecasted)
#define RCC ((RCC_RegDef_t*)RCC_BASE_ADDR)




//                            Clock ENABLE Macros for Peripherals                       //

//Clock enable for GPIO
#define GPIOA_PCLK_EN()          (RCC->AHB1ENR |= (1 << 0))


//Clock enable for I2C 


//Clock enable for SPI


//Clock enable for USART





//                            Clock DISABLE Macros for Peripherals                       //



//General-purpose timers (TIM2 to TIM5)
typedef struct
{
    volatile uint32_t CR1; //Control Register1
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RESERVED0;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
    volatile uint32_t OR;

}GEN_TIMER_RegDef_t;


//Struct for I2C Registers
typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;

}I2C_RegDef_t;


//I2C base addresses to use with I2C_RegDef structure pointers (typecasted)
#define I2C1 ((I2C_RegDef_t*)I2C1_BASE_ADDR)
#define I2C2 ((I2C_RegDef_t*)I2C2_BASE_ADDR)
#define I2C3 ((I2C_RegDef_t*)I2C3_BASE_ADDR)

