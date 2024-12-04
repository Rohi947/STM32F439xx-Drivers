/*
 * stm32f439xx_usart_driver.c
 *
 *  Created on: Nov 19, 2024
 *      Author: mrohi
 */

#include "stm32f439xx_usart_driver.h"

//Baud Rate Calculation
void Set_BRR(USART_RegDef_t *pUSART, uint32_t BaudRate)
{
	uint32_t PCLK, usartdiv, div_mantissa, div_fraction;
	uint32_t tempreg = 0;

	if(pUSART == USART1)
	{
		PCLK = RCC_GetPCLK2Value();
	}
	else
	{
		PCLK = RCC_GetPCLK1Value();
	}

	//OVER8
	if(pUSART->CR1 & (0x1 << USART_CR1_OVER8))
	{
		usartdiv = ((25 * PCLK) / (2 * BaudRate ));
	}
	else
	{
		usartdiv = ((25 * PCLK) / (4 * BaudRate ));
	}

	div_mantissa = usartdiv/100;

	tempreg |= div_mantissa << 4;

	div_fraction = (usartdiv - (div_mantissa*100));

  //Calculate the final fractional
  if(pUSART->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  div_fraction = ((( div_fraction * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   div_fraction = ((( div_fraction * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  tempreg |= div_fraction;

  pUSART->BRR = tempreg;


}

//USART Clock Setup
void USART_ClockControl(USART_RegDef_t *pUSART, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSART == USART1)
		{
			USART1_CLOCK_EN();
		}
		else if(pUSART == USART2)
		{
			USART2_CLOCK_EN();
		}
		else if(pUSART == USART3)
		{
			USART3_CLOCK_EN();
		}
	}
	else
	{
		if(pUSART == USART1)
		{
			USART1_CLOCK_DI();
		}
		else if(pUSART == USART2)
		{
			USART2_CLOCK_DI();
		}
		else if(pUSART == USART3)
		{
			USART3_CLOCK_DI();
		}
	}
}

//USART Init and DeInit
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	USART_ClockControl(pUSARTHandle->pUSART, ENABLE);

	uint32_t tempreg;

	//Word Length
	tempreg = pUSARTHandle->USART_Config.Word_Length;
	pUSARTHandle->pUSART->CR1 |= (tempreg << USART_CR1_M);

	//Stop Bits
	tempreg = pUSARTHandle->USART_Config.Stop_Bits;
	pUSARTHandle->pUSART->CR2 |= (tempreg << USART_CR2_STOP);

	//Parity Control
	tempreg = pUSARTHandle->USART_Config.Parity_Control;
	if(tempreg == PARITY_CONTROL_DISABLED)
	{
		pUSARTHandle->pUSART->CR1 &= ~(0x1 << USART_CR1_PCE);
	}
	else if(tempreg == PARITY_CONTROL_EN_ODD)
	{
		pUSARTHandle->pUSART->CR1 |= (0x1 << USART_CR1_PCE);
		pUSARTHandle->pUSART->CR1 |= (0x1 << USART_CR1_PS);
	}
	else
	{
		pUSARTHandle->pUSART->CR1 |= (0x1 << USART_CR1_PCE);
		pUSARTHandle->pUSART->CR1 &= ~(0x1 << USART_CR1_PS);
	}

	//MODE
	tempreg = pUSARTHandle->USART_Config.Mode;
	if(tempreg == MODE_TX)
	{
		pUSARTHandle->pUSART->CR1 |= (0x1 << USART_CR1_TE);
		pUSARTHandle->pUSART->CR1 &= ~(0x1 << USART_CR1_RE);
	}
	else if(tempreg == MODE_RX)
	{
		pUSARTHandle->pUSART->CR1 |= (0x1 << USART_CR1_RE);
		pUSARTHandle->pUSART->CR1 &= ~(0x1 << USART_CR1_TE);
	}
	else
	{
		pUSARTHandle->pUSART->CR1 |= (0x1 << USART_CR1_TE);
		pUSARTHandle->pUSART->CR1 |= (0x1 << USART_CR1_RE);
	}

	//BAUD
	Set_BRR(pUSARTHandle->pUSART, pUSARTHandle->USART_Config.Baud);

	//HW Control
	tempreg = pUSARTHandle->USART_Config.HWControl;
	if(tempreg == USART_HW_FLOW_CTRL_CTS)
	{
		pUSARTHandle->pUSART->CR3 |= (0x1 << USART_CR3_CTSE);
	}
	else if(tempreg == USART_HW_FLOW_CTRL_RTS)
	{
		pUSARTHandle->pUSART->CR3 |= (0x1 << USART_CR3_RTSE);
	}
	else
	{
		pUSARTHandle->pUSART->CR3 |= (0x1 << USART_CR3_CTSE);
	}

}


void USART_DeInit(USART_RegDef_t *pUSART)
{
	if(pUSART == USART1)
	{
		RCC->APB2RSTR |= (0x1 << 4);
		RCC->APB2RSTR &= ~(0x1 << 4);
	}
	else if(pUSART == USART2)
	{
		RCC->APB1RSTR |= (0x1 << 17);
		RCC->APB1RSTR &= ~(0x1 << 17);
	}
	else if(pUSART == USART3)
	{
		RCC->APB1RSTR |= (0x1 << 18);
		RCC->APB1RSTR &= ~(0x1 << 18);
	}
}

//Blocking APIs

//USART Receive
void USART_ReceiveData(USART_Handle_t *pUSART, uint8_t *pRxBuffer, uint8_t len, uint8_t Slave_Address)
{
	for(uint32_t i = 0; i<len; i++)
	{
		//Wait untill RXNE
		while(!USART_GetFlagStatus(pUSART->pUSART, USART_FLAG_RXNE));

		//check if 9 bit or 8 bit
		if(pUSART->USART_Config.Word_Length == WORD_LENGTH_9)
		{
			//Check for Parity
			if(pUSART->USART_Config.Parity_Control == PARITY_CONTROL_DISABLED)
			{
				//9 Bits of data
				*((uint16_t*)pRxBuffer) = pUSART->pUSART->DR & (uint16_t)0x1ff;

				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//8 bits of data
				*pRxBuffer = (pUSART->pUSART->DR & (uint8_t)0xff);
				pRxBuffer++;
			}
		}
		else
		{
			//Check for Parity
			if(pUSART->USART_Config.Parity_Control == PARITY_CONTROL_DISABLED)
			{
				//8 Bits of data
				*pRxBuffer = (uint8_t)(pUSART->pUSART->DR & (uint8_t)0xff);
			}
			else
			{
				//7 bits of data
				*pRxBuffer = (uint8_t)(pUSART->pUSART->DR & (uint8_t)0x7f);
			}
			pRxBuffer++;
		}
		}
}

//USART Transmit
void USART_SendData(USART_Handle_t *pUSART, uint8_t *pTxBuffer, uint8_t len, uint8_t Slave_Address)
{
	uint16_t *pdata;

	for(uint32_t i = 0; i<len; i++)
	{
		//Check if TXE is empty
		while(USART_GetFlagStatus(pUSART->pUSART, USART_FLAG_TXE) != SET);

		if(pUSART->USART_Config.Word_Length == WORD_LENGTH_9)
		{
			pdata = (uint16_t*)pTxBuffer;

			pUSART->pUSART->DR = (*pdata & (uint16_t)(0x1ff));

			if(pUSART->USART_Config.Parity_Control == PARITY_CONTROL_DISABLED)
			{
				//All the 9 bits are data so data is in 2 registers so increment tx buffer twice.
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Only 8 bits of data and 1 bit is pa rity set by hardware. So increment tx buffer once
				pTxBuffer++;
			}
		}
		else
		{
			pUSART->pUSART->DR = (*pTxBuffer & (uint8_t)0xff);
			pTxBuffer++;
		}
	}
	while(!USART_GetFlagStatus(pUSART->pUSART, USART_FLAG_TC));

}


//Interrupt Based

//USART Receive
uint8_t USART_ReceiveData_IT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint8_t len)
{
	uint8_t rxstate = pUSARTHandle->TxRxState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->TxRxState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSART->DR;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSART->CR1 |= ( 1 << USART_CR1_RXNEIE);

	}

	return rxstate;
}

//USART Transmit
uint8_t USART_SendData_IT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint8_t len)
{
	uint8_t txstate = pUSARTHandle->TxRxState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxRxState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSART->CR1 |= ( 1 << USART_CR1_TXEIE);


		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSART->CR1 |= ( 1 << USART_CR1_TCIE);


	}

	return txstate;
}


//IRQ
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
				*NVIC_ICER0 |= (1 << IRQNumber);
			}
			else if(IRQNumber > 31 && IRQNumber <= 63 )
			{
				*NVIC_ICER1 |= (1 << (IRQNumber%32));
			}
			else if(IRQNumber > 63 && IRQNumber <= 95 )
			{
				*NVIC_ICER2 |= (1 << IRQNumber%64);
			}
		}
}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint32_t temp1 = (IRQNumber / 4);
	uint32_t temp2 = (IRQNumber % 4);
	uint32_t shift = (8 * temp2) + (8 - NO_PR_BITS_IMPLIMENTED);
	*(NVIC_IPR + (temp1)) |= (IRQPriority << shift*8);
}


void USART_IRQHandling(USART_Handle_t *pUSART)
{
	uint32_t temp1, temp2, temp3;
	uint16_t* pdata;

	//Check if the interrupt is because of TC Flag
	temp1 = USART_GetFlagStatus(pUSART->pUSART, USART_FLAG_TC);

	//Check for TCIE interrupt enable
	temp2 = pUSART->pUSART->CR1 & (0x1 << USART_CR1_TCIE);

	//if temp1 and temp 2 are set then the interrput is because of TC
	if(temp1 && temp2)
	{
		//Transmission is complete so close the transmission and call Application callback if len = 0
		if(pUSART->TxRxState == USART_BUSY_IN_TX)
		{
			if(!pUSART->TxLen)
			{
				pUSART->pUSART->SR &= ~(0x1 << USART_FLAG_TC);

				pUSART->pUSART->CR1 &= ~(0x1 << USART_CR1_TCIE);

				pUSART->TxRxState = USART_READY;

				pUSART->pTxBuffer = NULL;

				pUSART->TxLen = 0;

				USART_ApplicationEventCallback(pUSART, USART_EVENT_TX_CMPLT);
			}
		}
	}

	//Check if interrupt is because of TXE
	temp1 = USART_GetFlagStatus(pUSART->pUSART, USART_FLAG_TXE);

	temp2 = (pUSART->pUSART->CR1 & (0x1 << USART_CR1_TXEIE));

	if(temp1 && temp2)
	{
		//TXE is set

		if(pUSART->TxRxState == USART_BUSY_IN_TX)
		{
			//Send data till len = 0
			if(pUSART->TxLen > 0)
			{
				//Check if it is 9 bit length or 8
				if(pUSART->USART_Config.Word_Length == WORD_LENGTH_9)
				{
					pdata = (uint16_t*)pUSART->pTxBuffer;
					pUSART->pUSART->DR = (*pdata & (uint16_t)(0x1FF));

					if(pUSART->USART_Config.Parity_Control == PARITY_CONTROL_DISABLED)
					{
						pUSART->pTxBuffer++;
						pUSART->pTxBuffer++;

						pUSART->TxLen--;
						pUSART->TxLen--;
					}
					else
					{
						pUSART->pTxBuffer++;
						pUSART->TxLen--;
					}
				}
				else
				{
					//8 Bit Transfer
					pUSART->pUSART->DR = (*pUSART->pTxBuffer & (uint8_t)0xff);

					pUSART->pTxBuffer++;

					pUSART->TxLen--;
				}
			}
			if(pUSART->TxLen == 0)
			{
				//Clear TXEIE BIT
				pUSART->pUSART->CR1 &= ~(0x1 << USART_CR1_TXEIE);
			}
		}
	}

	//Check fi the interpput is becasue of the RXNE

	temp1 = USART_GetFlagStatus(pUSART->pUSART, USART_FLAG_RXNE);

	temp2 = pUSART->pUSART->CR1 & (0x1 << USART_CR1_RXNEIE);

	if(temp1 && temp2)
	{
		//Interrupt because of RXNE
		if(pUSART->TxRxState == USART_BUSY_IN_RX)
		{
			//Check if RXLen is greater than 0
			if(pUSART->RxLen > 0)
			{
				//Check for data length
				if(pUSART->USART_Config.Word_Length == WORD_LENGTH_9)
				{
					//check for parity
					if(pUSART->USART_Config.Parity_Control == PARITY_CONTROL_DISABLED)
					{
						*((uint16_t*)pUSART->pRxBuffer) = (pUSART->pUSART->DR & (uint16_t)0x1FF);

						pUSART->pRxBuffer++;
						pUSART->pRxBuffer++;

						pUSART->RxLen--;
						pUSART->RxLen--;

					}
					else
					{
						*pUSART->pRxBuffer = (pUSART->pUSART->DR & (uint8_t)0xFF);
						pUSART->pRxBuffer++;
						pUSART->RxLen--;
					}
				}
				else
				{
					if(pUSART->USART_Config.Parity_Control == PARITY_CONTROL_DISABLED)
					{
						*pUSART->pRxBuffer = (pUSART->pUSART->DR & (uint8_t)0xFF);
						pUSART->pRxBuffer++;
						pUSART->RxLen--;

					}
					else
					{
						*pUSART->pRxBuffer = (pUSART->pUSART->DR & (uint8_t)0x7F);
						pUSART->pRxBuffer++;
						pUSART->RxLen--;
					}
				}
			}
			if(pUSART->RxLen == 0)
			{
				//Clear RXNEIE
				pUSART->pUSART->CR1 &= ~(USART_CR1_RXNEIE);
				pUSART->TxRxState = USART_READY;
				USART_ApplicationEventCallback(pUSART, USART_EVENT_RX_CMPLT);
			}
		}
	}

	/*************************Check for CTS flag ********************************************/
	//Note : CTS feature is not applicable for UART4 and UART5

		//Implement the code to check the status of CTS bit in the SR
		temp1 = pUSART->pUSART->SR & ( 1 << USART_FLAG_CTS);

		//Implement the code to check the state of CTSE bit in CR1
		temp2 = pUSART->pUSART->CR3 & ( 1 << USART_CR3_CTSE);

		//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
		temp3 = pUSART->pUSART->CR3 & ( 1 << USART_CR3_CTSIE);


		if(temp1  && temp2 )
		{
			//Implement the code to clear the CTS flag in SR
			pUSART->pUSART->SR &=  ~( 1 << USART_FLAG_CTS);

			//this interrupt is because of cts
			USART_ApplicationEventCallback(pUSART,USART_EVENT_CTS);
		}

	/*************************Check for IDLE detection flag ********************************************/

		//Implement the code to check the status of IDLE flag bit in the SR
		temp1 = pUSART->pUSART->SR & ( 1 << USART_FLAG_IDLE);

		//Implement the code to check the state of IDLEIE bit in CR1
		temp2 = pUSART->pUSART->CR1 & ( 1 << USART_CR1_IDLEIE);


		if(temp1 && temp2)
		{
			//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
			temp1 = pUSART->pUSART->SR &= ~( 1 << USART_FLAG_IDLE);

			//this interrupt is because of idle
			USART_ApplicationEventCallback(pUSART,USART_EVENT_IDLE);
		}

	/*************************Check for Overrun detection flag ********************************************/

		//Implement the code to check the status of ORE flag  in the SR
		temp1 = pUSART->pUSART->SR & USART_FLAG_ORE;

		//Implement the code to check the status of RXNEIE  bit in the CR1
		temp2 = pUSART->pUSART->CR1 & USART_CR1_RXNEIE;


		if(temp1  && temp2 )
		{
			//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

			//this interrupt is because of Overrun error
			USART_ApplicationEventCallback(pUSART,USART_ERR_ORE);
		}



	/*************************Check for Error Flag ********************************************/

	//Noise Flag, Overrun error and Framing Error in multibuffer communication
	//We dont discuss multibuffer communication in this course. please refer to the RM
	//The blow code will get executed in only if multibuffer mode is used.

		temp2 =  pUSART->pUSART->CR3 & ( 1 << USART_CR3_EIE) ;

		if(temp2 )
		{
			temp1 = pUSART->pUSART->SR;
			if(temp1 & ( 1 << USART_FLAG_FE))
			{
				/*
					This bit is set by hardware when a de-synchronization, excessive noise or a break character
					is detected. It is cleared by a software sequence (an read to the USART_SR register
					followed by a read to the USART_DR register).
				*/
				USART_ApplicationEventCallback(pUSART,USART_ERR_FE);
			}

			if(temp1 & ( 1 << USART_FLAG_NF) )
			{
				/*
					This bit is set by hardware when noise is detected on a received frame. It is cleared by a
					software sequence (an read to the USART_SR register followed by a read to the
					USART_DR register).
				*/
				USART_ApplicationEventCallback(pUSART,USART_ERR_NE);
			}

			if(temp1 & ( 1 << USART_FLAG_ORE) )
			{
				USART_ApplicationEventCallback(pUSART,USART_ERR_ORE);
			}
		}

}


//Other Peripheral Control APIs
void USART_PeripheralControl(USART_RegDef_t *pUSART, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSART->CR1 |= (0x1 << USART_CR1_UE);
	}
	else
	{
		pUSART->CR1 &= ~(0x1 << USART_CR1_UE);
	}
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSART, uint32_t FlagName)
{
	if(pUSART->SR & (0x1 << FlagName))
	{
		return SET;
	}
	else
	{
		return RESET;
	}
}

//Application Callback
void USART_ApplicationEventCallback(USART_Handle_t *pUSART, uint8_t AppEvent);


