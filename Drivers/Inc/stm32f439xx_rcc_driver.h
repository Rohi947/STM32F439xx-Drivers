/*
 * stm32f439xx_rcc_driver.h
 *
 *  Created on: Nov 20, 2024
 *      Author: mrohi
 */

#ifndef INC_STM32F439XX_RCC_DRIVER_H_
#define INC_STM32F439XX_RCC_DRIVER_H_

#include "stm32f439xx.h"

//APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//APB2 clock value
uint32_t RCC_GetPCLK2Value(void);

//PLL Clock
uint32_t  RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F439XX_RCC_DRIVER_H_ */
