/*
 * stm32f439xx_i2c_driver.c
 *
 *  Created on: Nov 6, 2024
 *      Author: mrohi
 */
#include "stm32f439xx_i2c_driver.h"

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);


uint16_t AHB1_Prescaler_Value[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_Prescaler_Value[4] = {2, 4, 8, 16};
uint32_t dummy_read = 0;


void I2C_ClockControl(I2C_RegDef_t *pI2C, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2C == I2C1)
		{
			I2C1_CLOCK_EN();
		}
		else if(pI2C == I2C2)
		{
			I2C2_CLOCK_EN();
		}
		else if(pI2C == I2C3)
		{
			I2C3_CLOCK_EN();
		}
	}
	else
	{
		if(pI2C == I2C1)
		{
			I2C1_CLOCK_DI();
		}
		else if(pI2C == I2C2)
		{
			I2C2_CLOCK_DI();
		}
		else if(pI2C == I2C3)
		{
			I2C3_CLOCK_DI();
		}
	}
}


//I2C Init and DeInit
void I2C_Init(I2C_Handle_t *pI2CHandle)
{

	//Enable Clock
	I2C_ClockControl(pI2CHandle->pI2Cx, ENABLE);

	//Temp Register
	uint32_t tempreg = 0;


	//CR1
	tempreg = (pI2CHandle->I2C_Config.ACK_Control << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 |= tempreg;

	//CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 |= (tempreg & (0x3f));

	//OAR1
	tempreg = 0;
	tempreg = pI2CHandle->I2C_Config.Device_Address << I2C_OAR1_ADD_7_BIT;

	//reserved
	tempreg |= (0x1 << 14);

	pI2CHandle->pI2Cx->OAR1 |= tempreg;

	//CCR
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.SCL_Speed <= I2C_SCL_SPEED_SM)
	{
		ccr_value |= RCC_GetPCLK1Value()/(2*pI2CHandle->I2C_Config.SCL_Speed);
		tempreg |= (ccr_value * 0xfff);
	}
	else
	{
		tempreg |= (0x1 << I2C_CCR_Speed_MODE);
		tempreg |= (pI2CHandle->I2C_Config.Duty_Cycle << I2C_CCR_DUTY);

		if(pI2CHandle->I2C_Config.Duty_Cycle == I2C_FM_DUTY_2)
		{
			ccr_value |= RCC_GetPCLK1Value()/(3*pI2CHandle->I2C_Config.SCL_Speed);
		}
		else
		{
			ccr_value |= RCC_GetPCLK1Value()/(25*pI2CHandle->I2C_Config.SCL_Speed);
		}
		tempreg |= (ccr_value & 0xfff);
	}
	pI2CHandle->pI2Cx->CCR |= tempreg;

	//TRISE
	if(pI2CHandle->I2C_Config.SCL_Speed <= I2C_SCL_SPEED_SM)
	{
		tempreg = (RCC_GetPCLK1Value()/1000000U)+1;
	}
	else
	{
		tempreg = ((RCC_GetPCLK1Value() * 300)/1000000000U)+1;
	}
	pI2CHandle->pI2Cx->TRISE |= tempreg & (0x3F);
}

void I2C_DeInit(I2C_RegDef_t *pI2C)
{
	if(pI2C == I2C1)
	{
		RCC->APB1RSTR |= (1<<21);
		RCC->APB1RSTR &= ~(1<<21);
	}
	else if(pI2C == I2C2)
	{
		RCC->APB1RSTR |= (1<<22);
		RCC->APB1RSTR &= ~(1<<22);
	}
	else if(pI2C == I2C3)
	{
		RCC->APB1RSTR |= (1<<23);
		RCC->APB1RSTR &= ~(1<<23);

	}
}

//Blocking APIs

//I2C Receive
void I2C_MasterReceiveData(I2C_Handle_t *pI2C, uint8_t *pRxBuffer, uint8_t len, uint8_t Slave_Address)
{
	//Enable ACK
	pI2C->pI2Cx->CR1 |= (0x1<<I2C_CR1_ACK);

	//Start Bit
	pI2C->pI2Cx->CR1 |= (1<<I2C_CR1_START);

	//EV5
	//Wait for SB = 1
	while(!(I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR1_SB)));

	//Clear SB
	dummy_read = pI2C->pI2Cx->SR1;

	//Write Slave Address to DR
	Slave_Address = Slave_Address << 1;
	Slave_Address |= (0x1);
	pI2C->pI2Cx->DR = Slave_Address;

	//EV6
	//Wait for ADDR = 1;
	while(!(I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR1_ADDR)));

	if(len == 1 || len == 2)
	{
		//Disable ACK
		pI2C->pI2Cx->CR1 &= ~(0x1<<I2C_CR1_ACK);

		if(len == 2)
		{
			//Set POS High
			pI2C->pI2Cx->CR1 |= (0x1 << I2C_CR1_POS);
		}
	}

	//Clear ADDR
	dummy_read = pI2C->pI2Cx->SR1;
	dummy_read = pI2C->pI2Cx->SR2;

	while(len > 2)
	{
		//Wait till RxNE is set
		while(!(I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR1_RxNE)));

		if(len == 3)
			{

				//Wait till BTF is set
				while(!(I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR1_BTF)));

				//Set Ack low
				pI2C->pI2Cx->CR1 &= ~(0x1<<I2C_CR1_ACK);
			}

		//Read into DR
		*pRxBuffer = pI2C->pI2Cx->DR;

		pRxBuffer++;
		len--;
	}



	//Wait till RxNE is set
	while(!(I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR1_RxNE)));

	if(len != 1)
	{
		//Wait till BTF is set
		while(!(I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR1_BTF)));


		if(pI2C->Sr == 1)
		{
			//Set Start High
			pI2C->pI2Cx->CR1 |= (I2C_STOP << I2C_CR1_START);
			pI2C->mode = I2C_Mode_Master;
		}
		else
		{
			//Set Stop High
			pI2C->pI2Cx->CR1 |= (I2C_STOP << I2C_CR1_STOP);
			pI2C->mode = I2C_Mode_Slave;
		}
		*pRxBuffer = pI2C->pI2Cx->DR;

		pRxBuffer++;
	}
	else
	{
		if(pI2C->Sr == 1)
		{
			//Set Start High
			pI2C->pI2Cx->CR1 |= (I2C_STOP << I2C_CR1_START);
			pI2C->mode = I2C_Mode_Master;
		}
			else
		{
			//Set Stop High
			pI2C->pI2Cx->CR1 |= (I2C_STOP << I2C_CR1_STOP);
			pI2C->mode = I2C_Mode_Slave;
		}
	}

	*pRxBuffer = pI2C->pI2Cx->DR;

	pRxBuffer++;

	*pRxBuffer = '\0';


}

//I2C Transmit
void I2C_MasterSendData(I2C_Handle_t *pI2C, uint8_t *pTxBuffer, uint8_t len, uint8_t Slave_Address)
{
	//Start Generation
	//printf("Waiting for Start");
	pI2C->pI2Cx->CR1 |= (0x1 << I2C_CR1_START);

	//printf("Waiting for EV5");
	//Check for EV5(SB=1)
	//Wait till SB is set.
	while(!(I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR1_SB)));

	//Clear SB
	dummy_read = pI2C->pI2Cx->SR1;

	//Execute Address Phase
	I2C_ExecuteAddressPhaseWrite(pI2C->pI2Cx, Slave_Address);

	//printf("Waiting for EV6");
	//Check for EV6
	//Wait till ADDR is set.
	while(!(I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR1_ADDR)));

	//Clear ADDR by reading SR2
	dummy_read = pI2C->pI2Cx->SR1;
	dummy_read = pI2C->pI2Cx->SR2;

	//Data Phase
	while(len > 0)
	{
		//printf("Waiting for EV8_1");
		//Check for EV8_1
		//Wait till TxE is Set
		while(!(I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR1_TxE)));

		//printf("Writing data to DR");
		//Write to DR
		pI2C->pI2Cx->DR = *pTxBuffer;

		//Decrease Len
		len--;
		//Locate to next data
		pTxBuffer++;
	}
	//printf("Waiting for EV8_2");
	//Check for EV8_2
	//Wait till BTF is set
	while(!(I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR1_BTF) && I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR1_TxE)));

	//printf("Stop");
	//Generate Stop/Restart Condition
	if(pI2C->Sr == 1)
	{
		//Set Start High
		pI2C->pI2Cx->CR1 |= (I2C_STOP << I2C_CR1_START);
		pI2C->mode = I2C_Mode_Master;
	}
	else
	{
		//Set STOP High
		pI2C->pI2Cx->CR1 |= (I2C_STOP << I2C_CR1_STOP);
		pI2C->mode = I2C_Mode_Slave;
	}
}


//I2C Receive
uint8_t I2C_MasterReceiveData_IT(I2C_Handle_t *pI2C, uint8_t *pRxBuffer, uint8_t len, uint8_t Slave_Address)
{
	//Check the state
	uint8_t busystate = pI2C->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{

		pI2C->pRxBuffer = pRxBuffer;
		pI2C->RxLen = len;
		pI2C->DevAddr = Slave_Address;
		pI2C->TxRxState = I2C_BUSY_IN_RX;
		pI2C->RxSize = len;
		pI2C->Sr = pI2C->Sr;

		//Start Condition
		pI2C->pI2Cx->CR1 |= (0x1 << I2C_CR1_START);
		pI2C->mode = I2C_Mode_Master;

		//ITBUFEN Enable
		pI2C->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITBUFEN);

		//ITEVFEN Enable
		pI2C->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITEVFEN);

		//ITERRN Enable
		pI2C->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITERREN);

		//ACK Enable
		//pI2C->pI2Cx->CR1 |= (0x1 << I2C_CR1_ACK);
	}

	return busystate;
}

//I2C Transmit
uint8_t I2C_MasterSendData_IT(I2C_Handle_t *pI2C, uint8_t *pTxBuffer, uint8_t len, uint8_t Slave_Address)
{
	//Check the state
	uint8_t busystate = pI2C->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2C->pTxBuffer = pTxBuffer;
		pI2C->TxLen = len;
		pI2C->DevAddr = Slave_Address;
		pI2C->TxRxState = I2C_BUSY_IN_TX;
		pI2C->Sr = pI2C->Sr;

		//Start Condition
		pI2C->pI2Cx->CR1 |= (0x1 << I2C_CR1_START);
		pI2C->mode = I2C_Mode_Master;

		//ITBUFEN Enable
		pI2C->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITBUFEN);

		//ITEVFEN Enable
		pI2C->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITEVFEN);

		//ITERRN Enable
		pI2C->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITERREN);
	}

	return busystate;
}


//IRQ
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint32_t temp1 = (IRQNumber / 4);
	uint32_t temp2 = (IRQNumber % 4);
	uint32_t shift = (8 * temp2) + (8 - NO_PR_BITS_IMPLIMENTED);
	*(NVIC_IPR + (temp1)) |= (IRQPriority << shift*8);
}

//Handles both master and slave mode interrupts
void I2C1_EV_IRQHandling(I2C_Handle_t *pI2C)
{
	uint32_t temp1, temp2, temp3, dummy;

	// Check if ITEVFEN is set
	temp1 = (pI2C->pI2Cx->CR2 & (0x1 << I2C_CR2_ITEVFEN));

	// Check if ITBUFEN is set
	temp2 = (pI2C->pI2Cx->CR2 & (0x1 << I2C_CR2_ITBUFEN));

	//Handle for SB
	temp3 = (I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR1_SB));

	if(temp1 && temp3)
	{
		//Handle SB
		//Clear SB and Execute Address Phase

		dummy = pI2C->pI2Cx->SR1;

		if(pI2C->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2C->pI2Cx, pI2C->DevAddr);
		}
		else if(pI2C->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2C->pI2Cx, pI2C->DevAddr);
		}
		pI2C->mode = I2C_Mode_Master;
	}

	//Handle for ADDR
	temp3 = (I2C_GetFlagStatus(I2C1, I2C_FLAG_SR1_ADDR));

	if(temp1 && temp3)
	{
		//Handle ADDR
		//Check if Master
		if(pI2C->mode == I2C_Mode_Master)
		{
			//Check if in Rx
			if(pI2C->TxRxState == I2C_BUSY_IN_RX)
			{
				if(pI2C->RxLen == 1 || pI2C->RxLen == 2)
				{
					//Set Ack Low
					pI2C->pI2Cx->CR1 &= ~(0x1 << I2C_CR1_ACK);

					if(pI2C->RxLen == 2)
					{
						//Set Pos High
						pI2C->pI2Cx->CR1 |= (0x1 << I2C_CR1_POS);
					}
				}
				uint8_t new_dummy = pI2C->pI2Cx->DR;
			}
		}

		//Clear ADDR
		dummy = pI2C->pI2Cx->SR1;
		dummy = pI2C->pI2Cx->SR2;
	}

	//Handle for BTF
	temp3 = (I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR1_BTF));

	if(temp1 && temp3)
	{

		//Handle BTF
		//Check if in Tx
		if(pI2C->TxRxState == I2C_BUSY_IN_TX)
		{
			//Check if TxE is Set
			if(I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR1_TxE))
			{
				if(pI2C->TxLen == 0)
					{
						//1. generate the STOP condition
						pI2C->pI2Cx->CR1 |= (0x1 << I2C_CR1_STOP);


						pI2C->mode = I2C_Mode_Slave;

						//2. reset all the member elements of the handle structure.
						I2C_CloseSendData(pI2C);

						//3. notify the application about transmission complete
						I2C_ApplicationEventCallback(pI2C, I2C_EV_TX_CMPLT);

						uint8_t new_dummy = pI2C->pI2Cx->DR;
					}
			}
		}
		else if(pI2C->TxRxState == I2C_BUSY_IN_RX)
		{
			if(I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR1_RxNE))
			{
				if(pI2C->RxLen == 3)
				{
					//Set Ack Low
					pI2C->pI2Cx->CR1 &= ~(0x1 << I2C_CR1_ACK);

					*(pI2C->pRxBuffer) = pI2C->pI2Cx->DR;

					pI2C->RxLen--;

					pI2C->pRxBuffer++;
				}
				else if(pI2C->RxLen == 2)
				{
					//Generate Stop/Restart Condition
					if(pI2C->Sr == 1)
					{
						//Set Start High
						pI2C->pI2Cx->CR1 |= (I2C_STOP << I2C_CR1_START);

						*(pI2C->pRxBuffer) = pI2C->pI2Cx->DR;

						pI2C->RxLen--;

						pI2C->pRxBuffer++;

						*(pI2C->pRxBuffer) = pI2C->pI2Cx->DR;

						//Enable RxNE Interrupt ie ITBUFEN
						pI2C->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITBUFEN);
					}
					else
					{
						//Set STOP High
						pI2C->pI2Cx->CR1 |= (I2C_STOP << I2C_CR1_STOP);

						pI2C->mode = I2C_Mode_Slave;

						*(pI2C->pRxBuffer) = pI2C->pI2Cx->DR;

						pI2C->RxLen--;

						pI2C->pRxBuffer++;

						*(pI2C->pRxBuffer) = pI2C->pI2Cx->DR;

						//2. reset all the member elements of the handle structure.
						I2C_CloseReceiveData(pI2C);

						//3. notify the application about transmission complete
						I2C_ApplicationEventCallback(pI2C, I2C_EV_RX_CMPLT);
					}
				}
			}
		}
	}

	//Handle for STOPF
	temp3 = (I2C_GetFlagStatus(I2C1, I2C_FLAG_SR1_STOPF));

	//Only Applicable in Slave Mode
	if(temp1 && temp3)
	{
		//Handle STOPF
		pI2C->pI2Cx->CR1 |= 0x0000;

		//notify the application about transmission complete
		I2C_ApplicationEventCallback(pI2C, I2C_EV_TX_CMPLT);
	}

	//Handle for TxE
	temp3 = (I2C_GetFlagStatus(I2C1, I2C_FLAG_SR1_TxE));

	if(temp1 && temp2 && temp3)
	{
		//Handle TxE
		if(pI2C->mode == I2C_Mode_Master)
		{
			//TXE flag is set
			//We have to do the data transmission
			if(pI2C->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2C);
			}
		}else
		{
			//slave
			//make sure that the slave is really in transmitter mode
			if(I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2C, I2C_EV_DATA_REQ);
			}
		}
	}

	//Handle for RxNE
	temp3 = (I2C_GetFlagStatus(I2C1, I2C_FLAG_SR1_RxNE));

	if(temp1 && temp2 && temp3)
	{
		//Handle RxE
		//check device mode .
		if(pI2C->mode == I2C_Mode_Master)
		{
			//The device is master

			//RXNE flag is set
			if(pI2C->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2C);
			}

		}else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(!(I2C_GetFlagStatus(pI2C->pI2Cx, I2C_FLAG_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2C, I2C_EV_DATA_RCV);
			}
		}
	}

	temp1 = temp2 = temp3 = dummy = 0;

}

void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2C)
{
	if(pI2C->TxLen > 0)
	{
		//printf("Writing data to DR");
		//Write to DR
		pI2C->pI2Cx->DR = *(pI2C->pTxBuffer);

		//Decrease Len
		pI2C->TxLen--;
		//Locate to next data
		pI2C->pTxBuffer++;
	}

	if(pI2C->TxLen == 0)
	{
		//1. generate the STOP condition
		pI2C->pI2Cx->CR1 |= (0x1 << I2C_CR1_STOP);


		pI2C->mode = I2C_Mode_Slave;

		//2. reset all the member elements of the handle structure.
		I2C_CloseSendData(pI2C);

		//3. notify the application about transmission complete
		I2C_ApplicationEventCallback(pI2C, I2C_EV_TX_CMPLT);

		uint8_t new_dummy = pI2C->pI2Cx->DR;
	}
}


void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2C)
{
	if(pI2C->RxLen > 3)
	{
		*(pI2C->pRxBuffer) = pI2C->pI2Cx->DR;

		pI2C->RxLen--;

		pI2C->pRxBuffer++;
	}


	if(pI2C->RxLen == 1)
	{
		*(pI2C->pRxBuffer) = pI2C->pI2Cx->DR;

		//2. reset all the member elements of the handle structure.
		I2C_CloseReceiveData(pI2C);

		//3. notify the application about transmission complete
		I2C_ApplicationEventCallback(pI2C, I2C_EV_RX_CMPLT);
	}
}

//Other Peripheral Control APIs
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (0x1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(0x1 << I2C_CR1_PE);
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(FlagName < 16)
	{
		if(pI2Cx->SR1 & (0x1 << FlagName))
		{
			return FLAG_SET;
		}
		else
		{
			return FLAG_RESET;
		}
	}
	else
	{
		if(pI2Cx->SR2 & (0x1 << (FlagName-16)))
		{
			return FLAG_SET;
		}
		else
		{
			return FLAG_RESET;
		}
	}
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}


static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVFEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.ACK_Control == 0x1)
	{
		pI2CHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_ACK);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVFEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C, uint8_t AppEvent)
{

}
