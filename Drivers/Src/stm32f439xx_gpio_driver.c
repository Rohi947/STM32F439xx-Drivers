#include "stm32f439xx_gpio_driver.h"


/****************************************************************************************
 * @fn			-GPIO_ClockControl
 *
 * @brief		-Enables or Disables the clock
 *
 * @param[in]	-GPIO Handle
 * @param[in]	-Enable or Disable
 *
 * @return		-void
 */

//GPIO Clock Setup
void GPIO_ClockControl(GPIO_RegDef_t *pGPIO, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIO == GPIOA)
		{
			GPIOA_CLOCK_EN();
		}
		else if(pGPIO == GPIOB)
		{
			GPIOB_CLOCK_EN();
		}
		else if(pGPIO == GPIOC)
		{
			GPIOC_CLOCK_EN();
		}
		else if(pGPIO == GPIOD)
		{
			GPIOD_CLOCK_EN();
		}
		else if(pGPIO == GPIOE)
		{
			GPIOE_CLOCK_EN();
		}
		else if(pGPIO == GPIOF)
		{
			GPIOF_CLOCK_EN();
		}
		else if(pGPIO == GPIOG)
		{
			GPIOG_CLOCK_EN();
		}
		else if(pGPIO == GPIOH)
		{
			GPIOH_CLOCK_EN();
		}
		else if(pGPIO == GPIOI)
		{
			GPIOI_CLOCK_EN();
		}
	}

	else
	{
		if(pGPIO == GPIOA)
		{
			GPIOA_CLOCK_DI();
		}
		else if(pGPIO == GPIOB)
		{
			GPIOB_CLOCK_DI();
		}
		else if(pGPIO == GPIOC)
		{
			GPIOC_CLOCK_DI();
		}
		else if(pGPIO == GPIOD)
		{
			GPIOD_CLOCK_DI();
		}
		else if(pGPIO == GPIOE)
		{
			GPIOE_CLOCK_DI();
		}
		else if(pGPIO == GPIOF)
		{
			GPIOF_CLOCK_DI();
		}
		else if(pGPIO == GPIOG)
		{
			GPIOG_CLOCK_DI();
		}
		else if(pGPIO == GPIOH)
		{
			GPIOH_CLOCK_DI();
		}
		else if(pGPIO == GPIOI)
		{
			GPIOI_CLOCK_DI();
		}
	}
}

//GPIO Init and DeInit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	uint32_t temp1 = 0;

	//MODER
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//Configure FTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//Configure RTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//Configure FTSR AND RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//Configure the GPIO port for interrupt selection in SYSCFG_EXTICR
		uint32_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4);
		uint32_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);
		uint32_t value = GPIO_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_CLOCK_EN();
		SYSCFG->EXTICR[temp1] |= (value << 4 * temp2);

		//Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}
	temp = 0;
	//OTYPER
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_OPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//SPEED
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << 2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//PULL UP PULL DOWN
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//Alternate Function
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8);
	temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8);
	pGPIOHandle->pGPIOx->AFR[temp] &= ~(0xF << (4 * temp1));
	pGPIOHandle->pGPIOx->AFR[temp] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp1));
	temp = 0;
	temp1 = 0;


}

void GPIO_DeInit(GPIO_RegDef_t *pGPIO)
{
	if(pGPIO == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIO == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIO == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIO == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIO == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIO == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIO == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIO == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIO == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}


//GPIO Read
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIO->IDR >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIO)
{
	uint16_t value;
	value = (uint16_t)(pGPIO->IDR);
	return value;
}

//GPIO Write
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIO->ODR |= (Value << PinNumber);
	}
	else
	{
		pGPIO->ODR &= ~(0x1 << PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIO, uint16_t Value)
{
	pGPIO->ODR = Value;
}

//GPIO Toggle
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber)
{
	pGPIO->ODR ^= (1 << PinNumber);
}

//GPIO Interrupt Serve
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 63 )
		{
			*NVIC_ISER1 |= (1 << IRQNumber%32);
		}
		else if(IRQNumber > 63 && IRQNumber <= 95 )
		{
			*NVIC_ISER2 |= (1 << IRQNumber%64);
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 = (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 63 )
		{
			*NVIC_ICER1 = (1 << (IRQNumber%32));
		}
		else if(IRQNumber > 63 && IRQNumber <= 95 )
		{
			*NVIC_ICER2 = (1 << IRQNumber%64);
		}
	}
}

void GPIO_PriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint32_t temp1 = (IRQNumber / 4);
	uint32_t temp2 = (IRQNumber % 4);
	uint32_t shift = (8 * temp2) + (8 - NO_PR_BITS_IMPLIMENTED);
	*(NVIC_IPR + (temp1)) |= (IRQPriority << shift*8);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		//Clear
		EXTI->PR |= (1 << PinNumber);
	}
}
