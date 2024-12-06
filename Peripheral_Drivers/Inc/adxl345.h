/*
 * adxl345.h
 *
 *  Created on: Dec 5, 2024
 *      Author: mrohi
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#include "string.h"
#include "stm32f439xx.h"

/****************************************************CONFIG VARIABLES*****************************************************/
//uint8_t FIFO_SAMPLES = 0;
#define ADXL345_SLAVE_ADDRESS 											 (0x53)

/**************************************************************************************************************************
 * ************************************************* REGISTER DEFINITIONS **************************************************
 * ************************************************************************************************************************/


//To store Register Contents
typedef struct{
	uint8_t USE_TAP_ACT_FF;
	uint8_t USE_INT;
	uint8_t USE_FIFO;

	//Registers
	uint8_t DEVID;
	uint8_t THRESH_TAP;
	uint8_t OFSX;
	uint8_t OFSY;
	uint8_t OFSZ;
	uint8_t DUR;
	uint8_t LATENT;
	uint8_t WINDOW;
	uint8_t THRESH_ACT;
	uint8_t THRESH_INACT;
	uint8_t TIME_INACT;
	uint8_t ACT_INACT_CTL;
	uint8_t THRESH_FF;
	uint8_t TIME_FF;
	uint8_t TAP_AXES;
	uint8_t ACT_TAP_STATUS;
	uint8_t BW_RATE;
	uint8_t POWER_CTL;
	uint8_t INT_ENABLE;
	uint8_t INT_MAP;
	uint8_t INT_SOURCE;
	uint8_t DATA_FORMAT;
	int16_t DATAX;
	int16_t DATAY;
	int16_t DATAZ;
	uint8_t FIFO_CTL;
	uint8_t FIFO_STATUS;
}ADXL345_Config_t;


//Register Address
#define ADXL345_REG_DEVID               										(0x00)
#define ADXL345_REG_THRESH_TAP          										(0x1D)
#define ADXL345_REG_OFSX                										(0x1E)
#define ADXL345_REG_OFSY                										(0x1F)
#define ADXL345_REG_OFSZ                										(0x20)
#define ADXL345_REG_DUR                 										(0x21)
#define ADXL345_REG_LATENT              										(0x22)
#define ADXL345_REG_WINDOW              										(0x23)
#define ADXL345_REG_THRESH_ACT          										(0x24)
#define ADXL345_REG_THRESH_INACT        										(0x25)
#define ADXL345_REG_TIME_INACT          										(0x26)
#define ADXL345_REG_ACT_INACT_CTL       										(0x27)
#define ADXL345_REG_THRESH_FF           										(0x28)
#define ADXL345_REG_TIME_FF             										(0x29)
#define ADXL345_REG_TAP_AXES            										(0x2A)
#define ADXL345_REG_ACT_TAP_STATUS      										(0x2B)
#define ADXL345_REG_BW_RATE             										(0x2C)
#define ADXL345_REG_POWER_CTL           										(0x2D)
#define ADXL345_REG_INT_ENABLE          										(0x2E)
#define ADXL345_REG_INT_MAP             										(0x2F)
#define ADXL345_REG_INT_SOURCE          										(0x30)
#define ADXL345_REG_DATA_FORMAT         										(0x31)
#define ADXL345_REG_DATAX0              										(0x32)
#define ADXL345_REG_DATAX1              										(0x33)
#define ADXL345_REG_DATAY0             										 	(0x34)
#define ADXL345_REG_DATAY1              										(0x35)
#define ADXL345_REG_DATAZ0              										(0x36)
#define ADXL345_REG_DATAZ1              										(0x37)
#define ADXL345_REG_FIFO_CTL           										 	(0x38)
#define ADXL345_REG_FIFO_STATUS         										(0x39)



/**************************************************************************************************************************
 * ************************************************* CONFIG PARAMETERS **************************************************
 * ***********************************************************************************************************************/

/******************************************************ACT_INACT_CTL******************************************************/

#define ACT_INACT_CTL_ACT_AC_COUPLED											(0x01 << 7)
#define ACT_INACT_CTL_ACT_DC_COUPLED											(0x00 << 7)

#define ACT_INACT_CTL_ACT_X_ENABLE												(0x01 << 6)
#define ACT_INACT_CTL_ACT_X_DISABLE												(0x00 << 6)

#define ACT_INACT_CTL_ACT_Y_ENABLE												(0x01 << 5)
#define ACT_INACT_CTL_ACT_Y_DISABLE												(0x00 << 5)

#define ACT_INACT_CTL_ACT_Z_ENABLE												(0x01 << 4)
#define ACT_INACT_CTL_ACT_Z_DISABLE												(0x00 << 4)

#define ACT_INACT_CTL_INACT_AC_COUPLED											(0x01 << 3)
#define ACT_INACT_CTL_INACT_DC_COUPLED											(0x00 << 3)

#define ACT_INACT_CTL_INACT_X_ENABLE											(0x01 << 2)
#define ACT_INACT_CTL_INACT_X_DISABLE											(0x00 << 2)

#define ACT_INACT_CTL_INACT_Y_ENABLE											(0x01 << 1)
#define ACT_INACT_CTL_INACT_Y_DISABLE											(0x00 << 1)

#define ACT_INACT_CTL_INACT_Z_ENABLE											(0x01 << 0)
#define ACT_INACT_CTL_INACT_Z_DISABLE											(0x00 << 0)

/********************************************************TAP AXES*********************************************************/

#define TAP_AXES_SUPPRESS_DTAP													(0x01 << 3)
#define TAP_AXES_SUPPRESS_NO_DTAP												(0x00 << 3)

#define TAP_AXES_X_ENABLE														(0x01 << 2)

#define TAP_AXES_Y_ENABLE														(0x01 << 1)

#define TAP_AXES_Z_ENABLE														(0x01 << 0)

/*********************************************************BW RATE*********************************************************/

#define BW_RATE_NORMAL_PWR_OPR													(0x00 << 4)
#define BW_RATE_REDUCED_PWR_OPR													(0x01 << 4)

#define BW_RATE_RATE_25															(0x08 << 0)
#define BW_RATE_RATE_50															(0x09 << 0)
#define BW_RATE_RATE_100														(0x0A << 0)
#define BW_RATE_RATE_200														(0x0B << 0)
#define BW_RATE_RATE_400														(0x0C << 0)
#define BW_RATE_RATE_800														(0x0D << 0)
#define BW_RATE_RATE_1600														(0x0E << 0)
#define BW_RATE_RATE_3200														(0x0F << 0)

/*********************************************************POWER CTL*******************************************************/

#define POWER_CTL_MEASURE_MODE													(0x01 << 3)
#define POWER_CTL_STANDBY_MODE													(0x00 << 3)

/*********************************************************INT ENABLE*******************************************************/

#define INT_ENABLE_DATA_READY													(0x01 << 7)
#define INT_ENABLE_SINGLE_TAP													(0x01 << 6)
#define INT_ENABLE_DOUBLE_TAP													(0x01 << 5)
#define INT_ENABLE_ACT															(0x01 << 4)
#define INT_ENABLE_INACT														(0x01 << 3)
#define INT_ENABLE_FREE_FALL													(0x01 << 2)
#define INT_ENABLE_WATERMARK													(0x01 << 1)
#define INT_ENABLE_OVERRUN														(0x01 << 0)

/**********************************************************INT MAP********************************************************/


#define INT_MAP_DATA_READY_INT2													(0x01 << 7)
#define INT_MAP_SINGLE_TAP_INT2													(0x01 << 6)
#define INT_MAP_DOUBLE_TAP_INT2													(0x01 << 5)
#define INT_MAP_ACT_INT2														(0x01 << 4)
#define INT_MAP_INACT_INT2														(0x01 << 3)
#define INT_MAP_FREE_FALL_INT2													(0x01 << 2)
#define INT_MAP_WATERMARK_INT2													(0x01 << 1)
#define INT_MAP_OVERRUN_INT2													(0x01 << 0)

/********************************************************DATA Format******************************************************/

#define DATA_FORMAT_SELF_TEST													(0x01 << 7)
#define DATA_FORMAT_SPI_3_WIRE													(0x01 << 6)
#define DATA_FORMAT_INT_INVERT													(0x01 << 5)
#define DATA_FORMAT_FULL_RES													(0x01 << 3)
#define DATA_FORMAT_JUSTIFY_L													(0x01 << 2)
#define DATA_FORMAT_RANGE_2G													(0x00 << 0)
#define DATA_FORMAT_RANGE_4G													(0x01 << 0)
#define DATA_FORMAT_RANGE_8G													(0x02 << 0)
#define DATA_FORMAT_RANGE_16G													(0x03 << 0)

/********************************************************DATA Format******************************************************/

#define FIFO_CTL_MODE_BYPASS													(0x00 << 6)
#define FIFO_CTL_MODE_FIFO														(0x01 << 6)
#define FIFO_CTL_MODE_STREAM													(0x02 << 6)
#define FIFO_CTL_MODE_TRIGGER													(0x03 << 6)
#define FIFO_CTL_TRIGGER_INT2													(0x01 << 5)
//#define FIFO_CTL_SAMPLES														(FIFO_SAMPLES << 0)

/********************************************************* General *******************************************************/


//ADXL345 SDO/ALT Address Line Configuration
#define ADXL345_SDO_GND															DISABLE
#define ADXL345_SDO_VDD															ENABLE



/**************************************************************************************************************************
 * ********************************************************* API's *******************************************************
 * ***********************************************************************************************************************/

void ADXL345_init(ADXL345_Config_t *cReg, I2C_Handle_t *pI2C);

void ADXL345_ReadAxisData(ADXL345_Config_t *cReg, I2C_Handle_t *pI2C);

#endif /* INC_ADXL345_H_ */
