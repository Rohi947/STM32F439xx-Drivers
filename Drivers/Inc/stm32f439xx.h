#ifndef INC_STM32F439XX_H_
#define INC_STM32F439XX_H_

//Libraries
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/****************************************************************************************************************************
************************************************ Useful MACROS *************************************************************
***************************************************************************************************************************/

//Definitions
#define __vo															volatile

//Processor Specific Details
#define NO_PR_BITS_IMPLIMENTED											4

//Generic Macros
#define ENABLE															1
#define DISABLE															0
#define SET																ENABLE
#define RESET															DISABLE
#define GPIO_PIN_SET													SET
#define GPIO_PIN_RESET													RESET

//Priorities
#define NVIC_PR_0														0
#define NVIC_PR_1														1
#define NVIC_PR_2														2
#define NVIC_PR_3														3
#define NVIC_PR_4														4
#define NVIC_PR_5														5
#define NVIC_PR_6														6
#define NVIC_PR_7														7
#define NVIC_PR_8														8
#define NVIC_PR_9														9
#define NVIC_PR_10														10
#define NVIC_PR_11														11
#define NVIC_PR_12														12
#define NVIC_PR_13														13
#define NVIC_PR_14														14
#define NVIC_PR_15														15


//Macro for GPIO Port Code
#define GPIO_TO_CODE(x)													((x == GPIOA) ? 0:\
																		(x == GPIOB) ? 1:\
																		(x == GPIOC) ? 2:\
																		(x == GPIOD) ? 3:\
																		(x == GPIOE) ? 4:\
																		(x == GPIOF) ? 5:\
																		(x == GPIOG) ? 6:\
																		(x == GPIOH) ? 7:8)



//IRQ Numbers of STM32F439ZI

//GPIO
#define IRQ_NO_EXTI0													6
#define IRQ_NO_EXTI1													7
#define IRQ_NO_EXTI2													8
#define IRQ_NO_EXTI3													9
#define IRQ_NO_EXTI4													10
#define IRQ_NO_EXTI9_5													23
#define IRQ_NO_EXTI15_10												40

//SPI
#define IRQ_NO_SPI1														35
#define IRQ_NO_SPI2														36
#define IRQ_NO_SPI3														51

//I2C
#define IRQ_NO_I2C1_EV													31
#define IRQ_NO_I2C1_ER													32
#define IRQ_NO_I2C2_EV													33
#define IRQ_NO_I2C2_ER													34
#define IRQ_NO_I2C3_EV													72
#define IRQ_NO_I2C3_ER													73

//USART
#define IRQ_NO_USART1													37
#define IRQ_NO_USART2													38
#define IRQ_NO_USART3													39
#define IRQ_NO_USART4													52
#define IRQ_NO_USART5													53

/****************************************************************************************************************************
************************************************ Base Addresses ************************************************************
********************************************* Memory & Peripherals *************************************************************
***************************************************************************************************************************/



//Base Address of SRAM and Flash
#define FLASH_BASEADDR												(0x08000000U)
#define SRAM1_BASEADDR												(0x20000000U)
#define SRAM2_BASEADDR												(0x2001C000U)
#define ROM_BASEADDR												(0x1FFF0000U)
#define SRAM 														(SRAM1_BASEADDR)

//Base Address of Bus
#define APB1_BASEADDR												(0x40000000U)
#define APB2_BASEADDR												(0x40010000U)
#define AHB1_BASEADDR												(0x40020000U)
#define AHB2_BASEADDR												(0x50000000U)
#define AHB3_BASEADDR												(0x60000000U)

//Base Address of RCC
#define RCC_BASEADDR												(AHB1_BASEADDR + 0X3800U)

//Base Address of APB1 Peripherals
#define SPI2_BASEOFFSET												(0x3800U)
#define SPI2_BASEADDR												(APB1_BASEADDR + SPI2_BASEOFFSET)
#define SPI3_BASEOFFSET												(0x3C00U)
#define SPI3_BASEADDR												(APB1_BASEADDR + SPI3_BASEOFFSET)
#define USART2_BASEOFFSET											(0x4400U)
#define USART2_BASEADDR												(APB1_BASEADDR + USART2_BASEOFFSET)
#define USART3_BASEOFFSET											(0x4800U)
#define USART3_BASEADDR												(APB1_BASEADDR + USART3_BASEOFFSET)
#define UART4_BASEOFFSET											(0x4C00U)
#define UART4_BASEADDR												(APB1_BASEADDR + UART4_BASEOFFSET)
#define UART5_BASEOFFSET											(0x5000U)
#define UART5_BASEADDR												(APB1_BASEADDR + UART5_BASEOFFSET)
#define I2C1_BASEOFFSET												(0x5400U)
#define I2C1_BASEADDR												(APB1_BASEADDR + I2C1_BASEOFFSET)
#define I2C2_BASEOFFSET												(0x5800U)
#define I2C2_BASEADDR												(APB1_BASEADDR + I2C2_BASEOFFSET)
#define I2C3_BASEOFFSET												(0x5C00U)
#define I2C3_BASEADDR												(APB1_BASEADDR + I2C3_BASEOFFSET)

//Base Address of APB2 Peripherals
#define USART1_BASEOFFSET											(0x1000U)
#define USART1_BASEADDR												(APB2_BASEADDR + USART1_BASEOFFSET)
#define USART6_BASEOFFSET											(0x1400U)
#define USART6_BASEADDR												(APB2_BASEADDR + USART6_BASEOFFSET)
#define SPI1_BASEOFFSET												(0x3000U)
#define SPI1_BASEADDR												(APB2_BASEADDR + SPI1_BASEOFFSET)

#define SYSCFG_BASEOFFSET											(0x3800U)
#define SYSCFG_BASEADDR												(APB2_BASEADDR + SYSCFG_BASEOFFSET)
#define EXTI_BASEOFFSET												(0x3C00U)
#define EXTI_BASEADDR												(APB2_BASEADDR + EXTI_BASEOFFSET)

//Base Address of AHB1 Peripherals
#define GPIOA_BASEOFFSET											(0x0000U)
#define GPIOA_BASEADDR												(AHB1_BASEADDR + GPIOA_BASEOFFSET)
#define GPIOB_BASEOFFSET											(0x0400U)
#define GPIOB_BASEADDR												(AHB1_BASEADDR + GPIOB_BASEOFFSET)
#define GPIOC_BASEOFFSET											(0x0800U)
#define GPIOC_BASEADDR												(AHB1_BASEADDR + GPIOC_BASEOFFSET)
#define GPIOD_BASEOFFSET											(0x0C00U)
#define GPIOD_BASEADDR												(AHB1_BASEADDR + GPIOD_BASEOFFSET)
#define GPIOE_BASEOFFSET											(0x1000U)
#define GPIOE_BASEADDR												(AHB1_BASEADDR + GPIOE_BASEOFFSET)
#define GPIOF_BASEOFFSET											(0x1400U)
#define GPIOF_BASEADDR												(AHB1_BASEADDR + GPIOF_BASEOFFSET)
#define GPIOG_BASEOFFSET											(0x1800U)
#define GPIOG_BASEADDR												(AHB1_BASEADDR + GPIOG_BASEOFFSET)
#define GPIOH_BASEOFFSET											(0x1C00U)
#define GPIOH_BASEADDR												(AHB1_BASEADDR + GPIOH_BASEOFFSET)
#define GPIOI_BASEOFFSET											(0x2000U)
#define GPIOI_BASEADDR												(AHB1_BASEADDR + GPIOI_BASEOFFSET)


/***************************************************************************************************************************
************************************************ Register Definitions ************************************************************
***************************************************************************************************************************/


//Base Address(Structure) of RCC Registers
typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;

//Base Address of EXTI Registers
typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

//Base Address of SYSCFG Registers
typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

//Base Address(Structure) of GPIOA Registers
typedef struct{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
}GPIO_RegDef_t;

//Base Address of SPI Registers
typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;

//Base Address of I2C Registers
typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint16_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;

typedef struct{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_RegDef_t;


//Base Address of NVIC Registers

//Base Address of NVIC ISER Registers
#define NVIC_ISER0													((__vo uint32_t*)(0xE000E100))
#define NVIC_ISER1													((__vo uint32_t*)(0xE000E104))
#define NVIC_ISER2													((__vo uint32_t*)(0xE000E108))
#define NVIC_ISER3													((__vo uint32_t*)(0xE000E10C))
#define NVIC_ISER4													((__vo uint32_t*)(0xE000E110))
#define NVIC_ISER5													((__vo uint32_t*)(0xE000E114))
#define NVIC_ISER6													((__vo uint32_t*)(0xE000E118))
#define NVIC_ISER7													((__vo uint32_t*)(0xE000E11C))


//Base Address of NVIC ISER Registers
#define NVIC_ICER0													((__vo uint32_t*)(0xE000E180))
#define NVIC_ICER1													((__vo uint32_t*)(0xE000E184))
#define NVIC_ICER2													((__vo uint32_t*)(0xE000E188))
#define NVIC_ICER3													((__vo uint32_t*)(0xE000E18C))
#define NVIC_ICER4													((__vo uint32_t*)(0xE000E190))
#define NVIC_ICER5													((__vo uint32_t*)(0xE000E194))
#define NVIC_ICER6													((__vo uint32_t*)(0xE000E198))
#define NVIC_ICER7													((__vo uint32_t*)(0xE000E19C))

//Base Address of NVIC IPR Registers
#define NVIC_IPR													((__vo uint32_t*)(0xE000E400))


/***************************************************************************************************************************
********************************************** Peripheral Definitions ************************************************************
***************************************************************************************************************************/

//RCC Definition
#define RCC														    ((RCC_RegDef_t*)(RCC_BASEADDR))


//EXTI Definition
#define EXTI														((EXTI_RegDef_t*)(EXTI_BASEADDR))


//SYSCFG Definition
#define SYSCFG														((SYSCFG_RegDef_t*)(SYSCFG_BASEADDR))


//Peripheral Definitions
#define GPIOA														((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB														((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC														((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD														((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE														((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF														((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG														((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH														((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI														((GPIO_RegDef_t*)GPIOI_BASEADDR)


//SPI Definition
#define SPI1														((SPI_RegDef_t*)(SPI1_BASEADDR))
#define SPI2														((SPI_RegDef_t*)(SPI2_BASEADDR))
#define SPI3														((SPI_RegDef_t*)(SPI3_BASEADDR))


//I2C Definition
#define I2C1														((I2C_RegDef_t*)(I2C1_BASEADDR))
#define I2C2														((I2C_RegDef_t*)(I2C2_BASEADDR))
#define I2C3														((I2C_RegDef_t*)(I2C3_BASEADDR))

//USART Definitions
#define USART1														((USART_RegDef_t*)(USART1_BASEADDR))
#define USART2														((USART_RegDef_t*)(USART2_BASEADDR))
#define USART3														((USART_RegDef_t*)(USART3_BASEADDR))
#define USART4														((USART_RegDef_t*)(USART4_BASEADDR))
#define USART5														((USART_RegDef_t*)(USART5_BASEADDR))

/***************************************************************************************************************************
************************************************** Clock Enable ******************************************************************
***************************************************************************************************************************/

//Clock Enable Macros for GPIOx Peripherals

#define GPIOA_CLOCK_EN()											(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLOCK_EN()											(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLOCK_EN()											(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLOCK_EN()											(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLOCK_EN()											(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_CLOCK_EN()											(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_CLOCK_EN()											(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_CLOCK_EN()											(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_CLOCK_EN()											(RCC->AHB1ENR |= (1 << 8))

//Clock Enable Macros for I2Cx Peripherals

#define I2C1_CLOCK_EN()												((RCC->APB1ENR) |= (1 << 21))
#define I2C2_CLOCK_EN()												((RCC->APB1ENR) |= (1 << 22))
#define I2C3_CLOCK_EN()												((RCC->APB1ENR) |= (1 << 23))

//Clock Enable Macros for SPIx Peripherals

#define SPI1_CLOCK_EN()												((RCC->APB2ENR) |= (1 << 12))
#define SPI2_CLOCK_EN()												((RCC->APB1ENR) |= (1 << 14))
#define SPI3_CLOCK_EN()												((RCC->APB1ENR) |= (1 << 15))

//Clock Enable Macros for UARTx Peripherals

#define USART1_CLOCK_EN()											((RCC->APB2ENR) |= (1 << 4))
#define USART2_CLOCK_EN()											((RCC->APB1ENR) |= (1 << 17))
#define USART3_CLOCK_EN()											((RCC->APB1ENR) |= (1 << 18))
#define UART4_CLOCK_EN()											((RCC->APB1ENR) |= (1 << 19))
#define UART5_CLOCK_EN()											((RCC->APB1ENR) |= (1 << 20))
#define USART6_CLOCK_EN()											((RCC->APB2ENR) |= (1 << 5))

//Clock Enable Macros for SYSCFG Peripherals

#define SYSCFG_CLOCK_EN()											((RCC->APB2ENR) |= (1 << 14))

/***************************************************************************************************************************
************************************************** Clock Disable ******************************************************************
***************************************************************************************************************************/


//Clock Disable Macros for GPIOx Peripherals

#define GPIOA_CLOCK_DI()											(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLOCK_DI()											(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLOCK_DI()											(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLOCK_DI()											(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLOCK_DI()											(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_CLOCK_DI()											(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_CLOCK_DI()											(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_CLOCK_DI()											(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_CLOCK_DI()											(RCC->AHB1ENR &= ~(1 << 8))

//Clock Disable Macros for I2Cx Peripherals

#define I2C1_CLOCK_DI()												((RCC->APB1ENR) &= ~(1 << 21))
#define I2C2_CLOCK_DI()												((RCC->APB1ENR) &= ~(1 << 22))
#define I2C3_CLOCK_DI()												((RCC->APB1ENR) &= ~(1 << 23))

//Clock Disable Macros for SPIx Peripherals

#define SPI1_CLOCK_DI()												((RCC->APB2ENR) &= ~(1 << 12))
#define SPI2_CLOCK_DI()												((RCC->APB1ENR) &= ~(1 << 14))
#define SPI3_CLOCK_DI()												((RCC->APB1ENR) &= ~(1 << 15))

//Clock Disable Macros for UARTx Peripherals

#define USART1_CLOCK_DI()											((RCC->APB2ENR) &= ~(1 << 4))
#define USART2_CLOCK_DI()											((RCC->APB1ENR) &= ~(1 << 17))
#define USART3_CLOCK_DI()											((RCC->APB1ENR) &= ~(1 << 18))
#define UART4_CLOCK_DI()											((RCC->APB1ENR) &= ~(1 << 19))
#define UART5_CLOCK_DI()											((RCC->APB1ENR) &= ~(1 << 20))
#define USART6_CLOCK_DI()											((RCC->APB2ENR) &= ~(1 << 5))

//Clock Disable Macros for SYSCFG Peripherals

#define SYSCFG_CLOCK_DI()											((RCC->APB2ENR) &= ~(1 << 14))

/***************************************************************************************************************************
***************************************************** Reset ******************************************************************
***************************************************************************************************************************/


//Macros to reset GPIOx Peripherals
#define GPIOA_REG_RESET()											do{((RCC->AHB1RSTR) |= (1 << 0));	((RCC->AHB1RSTR) &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()											do{((RCC->AHB1RSTR) |= (1 << 1));	((RCC->AHB1RSTR) &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()											do{((RCC->AHB1RSTR) |= (1 << 2));	((RCC->AHB1RSTR) &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()											do{((RCC->AHB1RSTR) |= (1 << 3));	((RCC->AHB1RSTR) &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()											do{((RCC->AHB1RSTR) |= (1 << 4));	((RCC->AHB1RSTR) &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()											do{((RCC->AHB1RSTR) |= (1 << 5));	((RCC->AHB1RSTR) &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()											do{((RCC->AHB1RSTR) |= (1 << 6));	((RCC->AHB1RSTR) &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()											do{((RCC->AHB1RSTR) |= (1 << 7));	((RCC->AHB1RSTR) &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()											do{((RCC->AHB1RSTR) |= (1 << 8));	((RCC->AHB1RSTR) &= ~(1 << 8)); }while(0)


//Macros to reset USARTx Peripherals
#define USART1_REG_RESET()											do{((RCC->APB2RSTR) |= (1 << 4)); 	(((RCC->APB2RSTR) &= ~(1 << 4)));}while(0)
#define USART2_REG_RESET()											do{((RCC->APB1RSTR) |= (1 << 17)); 	(((RCC->APB1RSTR) &= ~(1 << 17));}while(0)
#define USART3_REG_RESET()											do{((RCC->APB1RSTR) |= (1 << 18)); 	(((RCC->APB1RSTR) &= ~(1 << 18));}while(0)
#define USART4_REG_RESET()											do{((RCC->APB1RSTR) |= (1 << 19)); 	(((RCC->APB1RSTR) &= ~(1 << 19));}while(0)
#define USART5_REG_RESET()											do{((RCC->APB1RSTR) |= (1 << 20)); 	(((RCC->APB1RSTR) &= ~(1 << 20));}while(0)

//Other Required Headers
#include "stm32f439xx_gpio_driver.h"
#include "stm32f439xx_spi_driver.h"
#include "stm32f439xx_i2c_driver.h"
#include "stm32f439xx_rcc_driver.h"
#include "stm32f439xx_usart_driver.h"

#endif /* INC_STM32F439XX_H_ */
