/*
 * stm32f439xx_gpio_driver.h
 *
 *  Created on: Apr 1, 2024
 *      Author: mrohi
 */

#ifndef INC_STM32F439XX_GPIO_DRIVER_H_
#define INC_STM32F439XX_GPIO_DRIVER_H_

#include "stm32f439xx.h"

//Structure to Configure Pin

typedef struct
{

	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_OPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

//Handle Structure for GPIO Pin

typedef struct
{
	//Pointer to hold the base address
	GPIO_RegDef_t *pGPIOx;   												//Holds the base address of the GPIO port
	GPIO_PinConfig_t GPIO_PinConfig;  										//Holds the Pin Configuration Details

}GPIO_Handle_t;

//GPIO PIN NUMBERS
#define GPIO_PIN_NO_0														0
#define GPIO_PIN_NO_1														1
#define GPIO_PIN_NO_2														2
#define GPIO_PIN_NO_3														3
#define GPIO_PIN_NO_4														4
#define GPIO_PIN_NO_5														5
#define GPIO_PIN_NO_6														6
#define GPIO_PIN_NO_7														7
#define GPIO_PIN_NO_8														8
#define GPIO_PIN_NO_9														9
#define GPIO_PIN_NO_10														10
#define GPIO_PIN_NO_11														11
#define GPIO_PIN_NO_12														12
#define GPIO_PIN_NO_13														13
#define GPIO_PIN_NO_14														14
#define GPIO_PIN_NO_15														15


//GPIO Pin Modes
#define GPIO_MODE_IN 														0
#define GPIO_MODE_OUT 														1
#define GPIO_MODE_ALTFN 													2
#define GPIO_MODE_ANALOG													3
#define GPIO_MODE_IT_FT														4
#define GPIO_MODE_IT_RT														5
#define GPIO_MODE_IT_RFT													6

//GPIO Output Type
#define GPIO_OP_TYPE_PP														0
#define GPIO_OP_TYPE_OD														1

//GPIO Output Speed
#define GPIO_SPEED_LOW														0
#define GPIO_SPEED_MEDIUM													1
#define GPIO_SPEED_FAST														2
#define GPIO_SPEED_HIGH														3

//GPIO Pin Pull Up or Pull Down
#define GPIO_NO_PUPD														0
#define GPIO_PIN_PU															1
#define GPIO_PIN_PD															2

//GPIO Alternate Function Modes
#define GPIO_AF0															0
#define GPIO_AF1															1
#define GPIO_AF2															2
#define GPIO_AF3															3
#define GPIO_AF4															4
#define GPIO_AF5															5
#define GPIO_AF6															6
#define GPIO_AF7															7
#define GPIO_AF8															8
#define GPIO_AF9															9
#define GPIO_AF10															10
#define GPIO_AF11															11
#define GPIO_AF12															12
#define GPIO_AF13															13
#define GPIO_AF14															14
#define GPIO_AF15															15

/******************************************************************************************************************************************************
 * 												API's SUPPORTED BY THIS DRIVER
 *****************************************************************************************************************************************************/

//GPIO Clock Setup
void GPIO_ClockControl(GPIO_RegDef_t *pGPIO, uint8_t EnorDi);

//GPIO Init and DeInit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIO);

//GPIO Read
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIO);

//GPIO Write
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIO, uint16_t Value);

//GPIO Toggle
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber);

//GPIO Interrupt Serve
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_PriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F439XX_GPIO_DRIVER_H_ */
