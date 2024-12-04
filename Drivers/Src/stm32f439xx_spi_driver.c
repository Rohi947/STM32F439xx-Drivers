#include "stm32f439xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);


//SPI Clock Setup
void SPI_ClockControl(SPI_RegDef_t *pSPI, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPI == SPI1)
		{
			SPI1_CLOCK_EN();
		}
		else if(pSPI == SPI2)
		{
			SPI2_CLOCK_EN();
		}
		else if(pSPI == SPI3)
		{
			SPI3_CLOCK_EN();
		}
	}
	else
	{
		if(pSPI == SPI1)
		{
			SPI1_CLOCK_DI();
		}
		else if(pSPI == SPI2)
		{
			SPI2_CLOCK_DI();
		}
		else if(pSPI == SPI3)
		{
			SPI3_CLOCK_DI();
		}
	}
}

//SPI Init and DeInit
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Clock Enable
	SPI_ClockControl(pSPIHandle->pSPIx, ENABLE);

	//Temp Register
	uint32_t tempreg = 0;

	//Mode
	tempreg |= (pSPIHandle->SPI_Config.Mode << SPI_CR1_MSTR);

	//Bus Config
	if(pSPIHandle->SPI_Config.Bus_config == SPI_Bus_Full_Duplex)
	{
		tempreg &= ~(0x1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.Bus_config == SPI_Bus_Half_Duplex)
	{
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.Bus_config == SPI_Bus_Simplex_Receive)
	{
		tempreg &= ~(0x1 << SPI_CR1_BIDIMODE);
		tempreg |= (0x1 << SPI_CR1_RX_ONLY);
	}

	//Configure the SPI serial Clock speed
	tempreg |= pSPIHandle->SPI_Config.Speed << SPI_CR1_BR;

	//Configure DFF
	tempreg |= pSPIHandle->SPI_Config.DFF << SPI_CR1_DFF;

	//Configure CPOL
	tempreg |= pSPIHandle->SPI_Config.CPOL << SPI_CR1_CPOL;

	//Configure CPHA
	tempreg |= pSPIHandle->SPI_Config.CPHA << SPI_CR1_CPHA;

	//Configure SSM
	tempreg |= pSPIHandle->SPI_Config.SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;

}

void SPI_DeInit(SPI_RegDef_t *pSPI)
{
	if(pSPI == SPI1)
	{
		RCC->APB2RSTR |= (1<<12);
		RCC->APB2RSTR &= ~(1<<12);
	}
	else if(pSPI == SPI2)
	{
		RCC->APB1RSTR |= (1<<14);
		RCC->APB1RSTR &= ~(1<<14);
	}
	else if(pSPI == SPI3)
	{
		RCC->APB1RSTR |= (1<<15);
		RCC->APB1RSTR &= ~(1<<15);
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/***********************************************************************************************************************
 **********************************************BLOCKING CALL************************************************************
***********************************************************************************************************************/

//SPI Transmit
void SPI_SendData(SPI_RegDef_t *pSPI, uint8_t *pTxBuffer, uint8_t len)
{
	while(len>0)
	{
		//1. Wait Till TXE is set
		while(SPI_GetFlagStatus(pSPI, SPI_FLAG_TXE) == FLAG_RESET);

		//2. Check the DFF bit
		if(pSPI->CR1 & (0x1 << SPI_CR1_DFF))
		{
			//16 Bit Data Format

			//1. Load the data into the DR
			pSPI->DR = *((uint16_t*)pTxBuffer);

			//2. Decrement len twice
			len--;
			len--;

			//3. Increment the Buffer to point to the next address.
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 Bit Data Format

			//1. Load the data into the DR
			pSPI->DR = *(pTxBuffer);

			//2. Decrement len
			len--;

			//3. Increment the Buffer to point to the next address.
			pTxBuffer++;
		}



	}
}


//SPI Receive
void SPI_Receive(SPI_RegDef_t *pSPI, uint8_t *pRxBuffer, uint8_t len)
{
	while(len>0)
	{
		while(SPI_GetFlagStatus(pSPI, SPI_FLAG_RXNE) == FLAG_RESET);

		if(pSPI->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 BIT
			*(uint16_t*)pRxBuffer = pSPI->DR;

			len--;
			len--;

			(uint16_t*)pRxBuffer++;
		}
		else
		{
			//8 Bit
			*pRxBuffer = pSPI->DR;

			len--;
			pRxBuffer++;
		}
	}
}


//Interrupt Based


//SPI Transmit
uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint8_t len)
{

	//IF THE SPI IS FREE

	uint8_t SPI_Status = pSPIHandle->TxState;

	if(SPI_Status != SPI_BUSY_IN_TX)
	{
		//1. SAVE THE TX BUFFER ADDRESS AND TX LEN

		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		//2. MARK THE SPI STATE AS BUSY IN TRANSMISSION SO THAT NO OTHER CODE
		//	CAN TAKE OVER SAME SPI PERIPHERAL UNTIL TRANSMISSION IS OVER

		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. ENABLE THE TXEIE CONTROL BIT TO GET INTERRUPT WHENEVER TXE FLAG IS SET IN SR

		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//4. DATA TRANSMISSION WILL BE HANDLED BY THE ISR CODE
	}

	return SPI_Status;
}



//SPI Receive
uint8_t SPI_Receive_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint8_t len)
{

	//IF THE SPI IS FREE

	uint8_t SPI_Status = pSPIHandle->RxState;

	if(SPI_Status != SPI_BUSY_IN_RX)
	{
		//1. SAVE THE TX BUFFER ADDRESS AND TX LEN

		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		//2. MARK THE SPI STATE AS BUSY IN TRANSMISSION SO THAT NO OTHER CODE
		//	CAN TAKE OVER SAME SPI PERIPHERAL UNTIL TRANSMISSION IS OVER

		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. ENABLE THE TXEIE CONTROL BIT TO GET INTERRUPT WHENEVER TXE FLAG IS SET IN SR

		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		//4. DATA TRANSMISSION WILL BE HANDLED BY THE ISR CODE
	}

	return SPI_Status;
}
//Other APIs
void SPI_PeripheralControl(SPI_RegDef_t *pSPI, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPI->CR1 |= (0x1 << SPI_CR1_SPE);
	}
	else
	{
		pSPI->CR1 &= ~(0x1 << SPI_CR1_SPE);
	}
}

void SSI_SSIConfig(SPI_RegDef_t *pSPI, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPI->CR2 |= (1<<SPI_CR1_SSI);
	}
	else
	{
		pSPI->CR2 &= ~(1<<SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPI, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPI->CR2 |= (1<<SPI_CR2_SSOE);
	}
	else
	{
		pSPI->CR2 &= ~(1<<SPI_CR2_SSOE);
	}
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint32_t temp1 = (IRQNumber / 4);
	uint32_t temp2 = (IRQNumber % 4);
	uint32_t shift = (8 * temp2) + (8 - NO_PR_BITS_IMPLIMENTED);
	*(NVIC_IPR + (temp1)) |= (IRQPriority << shift*8);
}



void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;

	//CHECK IF THE INTERRUPT IS BECAUSE OF TRANSMISSION

	temp1 = (pHandle->pSPIx->SR & (1 << SPI_SR_TXE));
	temp2 = (pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE));

	if(temp1 && temp2)
	{
		spi_txe_interrupt_handle(pHandle);
	}

	//CHECK IF THE INTERRUPT IS BECASUE OF RECEPTION
	temp1 = (pHandle->pSPIx->SR & (1 << SPI_SR_RXNE));
	temp2 = (pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE));

	if(temp1 && temp2)
	{
		spi_rxne_interrupt_handle(pHandle);
	}

	//CHECK IF THE INTERRUPT IS BECAUSE OF ERROR
	temp1 = (pHandle->pSPIx->SR & (1 << SPI_SR_OVR));
	temp2 = (pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE));

	if(temp1 && temp2)
	{
		spi_ovr_interrupt_handle(pHandle);
	}
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. Check the DFF bit
		if(pSPIHandle->pSPIx->CR1 & (0x1 << SPI_CR1_DFF))
		{
			//16 Bit Data Format

			//1. Load the data into the DR
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);

			//2. Decrement len twice
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;

			//3. Increment the Buffer to point to the next address.
			(uint16_t*)pSPIHandle->pTxBuffer++;
		}
		else
		{
			//8 Bit Data Format

			//1. Load the data into the DR
			pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);

			//2. Decrement len
			pSPIHandle->TxLen--;

			//3. Increment the Buffer to point to the next address.
			pSPIHandle->pTxBuffer++;
		}

		if(!pSPIHandle->TxLen)
		{
			//If TxLen == 0 then close the SPI Tx and inform the application that Tx is over

			SPI_CloseTransmission(pSPIHandle);

			//Inform the application
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

		}
}


static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
		if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 BIT
			*(uint16_t*)pSPIHandle->pRxBuffer = (uint16_t)pSPIHandle->pSPIx->DR;

			pSPIHandle->RxLen--;
			pSPIHandle->RxLen--;

			pSPIHandle->pRxBuffer++;
			pSPIHandle->pRxBuffer++;

		}
		else
		{
			//8 Bit
			*pSPIHandle->pRxBuffer = (uint8_t)pSPIHandle->pSPIx->DR;

			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer++;
		}

		if(!pSPIHandle->RxLen)
		{
			//If RxLen == 0 then close the SPI Rx and inform the application that Rx is over

			SPI_CloseReception(pSPIHandle);

			//Inform the application
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
		}
}


static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Clear OVR Flag
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	//Inform
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	//Clear the TXEIE bit
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

	//Reset the TxBuffer and TxLen
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;

	//Set the state to ready
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	//Clear the RXNEIE bit
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

	//Reset the TxBuffer and TxLen
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;

	//Set the state to ready
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{

}
