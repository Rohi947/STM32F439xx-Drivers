/*
 * adxl345.c
 *
 *  Created on: Dec 5, 2024
 *      Author: mrohi
 */

#include "adxl345.h"

void ADXL345_init(ADXL345_Config_t *cReg, I2C_Handle_t *pI2C)
{

	//Register Initialization of THRESH_TAP to TAP_AXES
	if(cReg->USE_TAP_ACT_FF == ENABLE)
	{
		uint8_t regbuffer[15] = {0};

		regbuffer[0] = ADXL345_REG_THRESH_TAP;
		memcpy(regbuffer+1, &cReg->THRESH_TAP, 14);

		I2C_MasterSendData(pI2C, regbuffer, 15, ADXL345_SLAVE_ADDRESS);
	}

	//Register Initialization of BW_RATE to INT MAP
	if(cReg->USE_INT == ENABLE)
	{
		uint8_t regbuffer[5] = {0};

		regbuffer[0] = ADXL345_REG_BW_RATE;
		memcpy(regbuffer+1, &cReg->BW_RATE, 4);
		I2C_MasterSendData(pI2C, regbuffer, 5, ADXL345_SLAVE_ADDRESS);
	}
	else
	{
		uint8_t regbuffer[3] = {0};

		regbuffer[0] = ADXL345_REG_BW_RATE;
		memcpy(regbuffer+1, &cReg->BW_RATE, 2);
		I2C_MasterSendData(pI2C, regbuffer, 3, ADXL345_SLAVE_ADDRESS);
	}

	//Register Initializartion of DATA_FORMAT
	do
	{
		uint8_t regbuffer[2] = {0};

		regbuffer[0] = ADXL345_REG_DATA_FORMAT;
		memcpy(regbuffer+1, &cReg->DATA_FORMAT, 1);
		I2C_MasterSendData(pI2C, regbuffer, 2, ADXL345_SLAVE_ADDRESS);
	}while(0);

	if(cReg->USE_FIFO == ENABLE)
	{
		uint8_t regbuffer[3] = {0};

		regbuffer[0] = ADXL345_REG_FIFO_CTL;
		memcpy(regbuffer+1, &cReg->FIFO_CTL, 2);
		I2C_MasterSendData(pI2C, regbuffer, 3, ADXL345_SLAVE_ADDRESS);
	}
}

//Read Axis Data
void ADXL345_ReadAxisData(ADXL345_Config_t *cReg, I2C_Handle_t *pI2C)
{
	uint8_t txbuffer = ADXL345_REG_DATAX0;
	uint8_t rxbuffer[6] = {0};

	I2C_MasterSendData(pI2C, &txbuffer, 1, ADXL345_SLAVE_ADDRESS);

	I2C_MasterReceiveData(pI2C, rxbuffer, 6, ADXL345_SLAVE_ADDRESS);

	cReg->DATAX = (rxbuffer[1]<<8 | rxbuffer[0]);
	cReg->DATAY = (rxbuffer[3]<<8 | rxbuffer[2]);
	cReg->DATAZ = (rxbuffer[5]<<8 | rxbuffer[4]);
}
