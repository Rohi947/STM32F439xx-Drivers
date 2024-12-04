/*
 * ds3231.h
 *
 *  Created on: Nov 21, 2024
 *      Author: mrohi
 */

#ifndef INC_DS3231_H_
#define INC_DS3231_H_

#include "stm32f439xx.h"

#define DS3231_ADDRESS																	(0b01101000)

/*
//DS3231 Register Definition
typedef struct{
	uint8_t Seconds;
	uint8_t Minutes;
	uint8_t Hours;
	uint8_t Day;
	uint8_t Date;
	uint8_t Month_Century;
	uint8_t Year;
	uint8_t ALM1_Seconds;
	uint8_t ALM1_Minutes;
	uint8_t ALM1_Hours;
	uint8_t ALM1_Day;
	uint8_t ALM1_Date;
	uint8_t ALM2_Minutes;
	uint8_t ALM2_Hours;
	uint8_t ALM2_Day;
	uint8_t ALM2_Date;
	uint8_t Control;
	uint8_t Cntrl_status;
	uint8_t Aging;
	uint8_t MSB_Temp;
	uint8_t LSB_Temp;
}DS_RTC_Result_t;

*/

//Holds the Time & Date information.

typedef struct{
	uint8_t Seconds;
	uint8_t Minutes;
	uint8_t Hours;
	uint8_t Hours_12_24;
	uint8_t Hours_AM_PM;
	uint8_t Day;
	uint8_t Date;
	uint8_t Month_Century;
	uint8_t Month_Century_Century;
	uint8_t Year;
	uint8_t ALM1_STATUS;
	uint8_t ALM1_RATE;
	uint8_t ALM1_Seconds;
	uint8_t ALM1_Minutes;
	uint8_t ALM1_Hours;
	uint8_t ALM1_Hours_12_24;
	uint8_t ALM1_Hours_AM_PM;
	uint8_t ALM1_DY_DT;
	uint8_t ALM1_Day_Date;
	uint8_t ALM2_STATUS;
	uint8_t ALM2_RATE;
	uint8_t ALM2_Minutes;
	uint8_t ALM2_Hours;
	uint8_t ALM2_Hours_12_24;
	uint8_t ALM2_Hours_AM_PM;
	uint8_t ALM2_DY_DT;
	uint8_t ALM2_Day_Date;
	uint8_t Control;
	uint8_t Cntrl_status;
	uint8_t Aging;
	uint8_t MSB_Temp;
	uint8_t LSB_Temp;
}DS_RTC_Config_t;

/**************************************************************************************************************************
 * ************************************************* REGISTER CONTENTS ****************************************************
 * ************************************************************************************************************************/

//Interrupt
#define INTCN_ENABLE																		(0x4)
#define INTCN_DISABLE																		~(0x4)

//00h
#define DS_RTC_POS_SECONDS_1S																0
#define DS_RTC_POS_SECONDS_10S																4

//01h
#define DS_RTC_POS_MINUTES_1S																0
#define DS_RTC_POS_MINUTES_10S																4

//02h
#define DS_RTC_POS_HOURS_1S																	0
#define DS_RTC_POS_HOURS_10S																4
#define DS_RTC_POS_HOURS_AM_PM_20S															5
#define DS_RTC_POS_HOURS_12_24																6

//03h
#define DS_RTC_POS_DAY_1S																	0

//04h
#define DS_RTC_POS_DATE_1S																	0
#define DS_RTC_POS_DATE_10S																	4

//05h
#define DS_RTC_POS_MONTH_1S																	0
#define DS_RTC_POS_MONTH_10S																4
#define DS_RTC_POS_MONTH_CENTURY															7

//06h
#define DS_RTC_POS_YEAR_1S																	0
#define DS_RTC_POS_YEAR_10S																	4

//07h
#define DS_RTC_POS_ALM1_SEC_1S																0
#define DS_RTC_POS_ALM1_SEC_10S																4
#define DS_RTC_POS_ALM1_SEC_A1M1															7

//08h
#define DS_RTC_POS_ALM1_MIN_1S																0
#define DS_RTC_POS_ALM1_MIN_10S																4
#define DS_RTC_POS_ALM1_MIN_A1M2															7

//09h
#define DS_RTC_POS_ALM1_HOUR_1S																0
#define DS_RTC_POS_ALM1_HOUR_10S															4
#define DS_RTC_POS_ALM1_HOUR_AM_PM_20S														5
#define DS_RTC_POS_ALM1_HOUR_12_24															6
#define DS_RTC_POS_ALM1_HOUR_A1M3															7

//0Ah
#define DS_RTC_POS_ALM1_DAY_DATE_1S															0
#define DS_RTC_POS_ALM1_DAY_DATE_10S														4
#define DS_RTC_POS_ALM1_DAY_DATE_DY_DT														6
#define DS_RTC_POS_ALM1_DAY_DATE_A1M4														7

//0Bh
#define DS_RTC_POS_ALM2_MIN_1S																0
#define DS_RTC_POS_ALM2_MIN_10S																4
#define DS_RTC_POS_ALM2_MIN_A2M2															7

//0Ch
#define DS_RTC_POS_ALM2_HOUR_1S																0
#define DS_RTC_POS_ALM2_HOUR_10S															4
#define DS_RTC_POS_ALM2_HOUR_AM_PM_20S														5
#define DS_RTC_POS_ALM2_HOUR_12_24															6
#define DS_RTC_POS_ALM2_HOUR_A2M3															7

//0Dh
#define DS_RTC_POS_ALM2_DAY_DATE_1S															0
#define DS_RTC_POS_ALM2_DAY_DATE_10S														4
#define DS_RTC_POS_ALM2_DAY_DATE_DY_DT														6
#define DS_RTC_POS_ALM2_DAY_DATE_A2M4														7

//0Eh
#define DS_RTC_POS_CTRL_A1IE																0
#define DS_RTC_POS_CTRL_A2IE																1
#define DS_RTC_POS_CTRL_INTCN																2
#define DS_RTC_POS_CTRL_RS1																	3
#define DS_RTC_POS_CTRL_RS2																	4
#define DS_RTC_POS_CTRL_CONV																5
#define DS_RTC_POS_CTRL_BBSQW																6
#define DS_RTC_POS_CTRL_EOSC																7

//0Fh
#define DS_RTC_POS_CTRL_ST_A1F																0
#define DS_RTC_POS_CTRL_ST_A2F																1
#define DS_RTC_POS_CTRL_ST_BSY																2
#define DS_RTC_POS_CTRL_ST_EN32KHz															3
#define DS_RTC_POS_CTRL_ST_OSF																7

//10h
#define DS_RTC_POS_AGING_DATA																0

//11h
#define DS_RTC_POS_TEMP_MSB																	0

//12h
#define DS_RTC_POS_TEMP_LSB																	0


//USEFUL MACROS
#define DS_RTC_HOURS_24																	0
#define DS_RTC_HOURS_12																	1

#define DS_RTC_HOURS_AM																	0
#define DS_RTC_HOURS_PM																	1

#define DS_RTC_ALM1_HOURS_AM															0
#define DS_RTC_ALM1_HOURS_PM															1

#define DS_RTC_ALM1_HOURS_24															0
#define DS_RTC_ALM1_HOURS_12															1

#define DS_RTC_ALM1_DT																	0
#define DS_RTC_ALM1_DY																	1

#define DS_RTC_ALM2_HOURS_AM															0
#define DS_RTC_ALM2_HOURS_PM															1

#define DS_RTC_ALM2_HOURS_24															0
#define DS_RTC_ALM2_HOURS_12															1

#define DS_RTC_ALM2_DT																	0
#define DS_RTC_ALM2_DY																	1

//Days
#define MONDAY																			1
#define TUESDAY																			2
#define WEDNESDAY																		3
#define THURSDAY																		4
#define FRIDAY																			5
#define SATURDAY																		6
#define SUNDAY																			7

//Alarm Mask Bits

//MIN_MATCH 	-> MIN AND SEC MATCH
//HOUR_MATCH 	-> HOUR AND MIN AND SEC MATCH
//DATA MATCH 	-> DATE AND HOUR AND MIN AND SEC MATCH
//DAY_MATCH 	-> DAY AND DATE AND HOUR AND MIN AND SEC MATCH


#define DS_RTC_ALM1_RATE_PER_SEC																	(0xF)
#define DS_RTC_ALM1_RATE_SEC_MATCH																	(0xE)
#define DS_RTC_ALM1_RATE_MIN_MATCH																	(0xC)
#define DS_RTC_ALM1_RATE_HOUR_MATCH																	(0x8)
#define DS_RTC_ALM1_RATE_DATE_MATCH																	(0x00)
#define DS_RTC_ALM1_RATE_DAY_MATCH																	(0x10)
#define DS_RTC_ALM2_RATE_PER_MIN																	(0x7)
#define DS_RTC_ALM2_RATE_MIN_MATCH																	(0x6)
#define DS_RTC_ALM2_RATE_HOUR_MATCH																	(0x4)
#define DS_RTC_ALM2_RATE_DATE_MATCH																	(0x0)
#define DS_RTC_ALM2_RATE_DAY_MATCH																	(0x8)


/**************************************************************************************************************************
 * ************************************************* Peripheral Definition ************************************************
 * ************************************************************************************************************************/

//DS3231 Definition
#define DS_REG_BASE_ADDR															(0x00)

#define DS_REG_Seconds																(0x00)
#define DS_REG_Minutes																(0x01)
#define DS_REG_Hours																(0x02)
#define DS_REG_Day																	(0x03)
#define DS_REG_Date																	(0x04)
#define DS_REG_Month_Century														(0x05)
#define DS_REG_Year																	(0x06)
#define DS_REG_ALM1_Seconds															(0x07)
#define DS_REG_ALM1_Minutes															(0x08)
#define DS_REG_ALM1_Hours															(0x09)
#define DS_REG_ALM1_Day																(0x0A)
#define DS_REG_ALM1_Date															(0x0B)
#define DS_REG_ALM2_Minutes															(0x0C)
#define DS_REG_ALM2_Hours															(0x0D)
#define DS_REG_ALM2_Day																(0x0E)
#define DS_REG_ALM2_Date															(0x0F)
#define DS_REG_Control																(0x10)
#define DS_REG_Cntrl_status															(0x11)
#define DS_REG_Aging																(0x12)
#define DS_REG_MSB_Temp																(0x13)
#define DS_REG_LSB_Temp																(0x14)


/**************************************************************************************************************************
 * ********************************************** Implemented API'S ******************************************************
 * ************************************************************************************************************************/

//Initialization
void DS_RTC_INIT(DS_RTC_Config_t* pRTC);

//Configures DS_RTC as a stopwatch
void DS_RTC_CONFIG_STOPWATCH(DS_RTC_Config_t* pRTC, I2C_Handle_t* pI2C);

//Sets the DS_RTC to current time
void DS_RTC_SET_CURRENT_TIME(DS_RTC_Config_t* pRTC, I2C_Handle_t* pI2C);

//Sets the DS_RTC to current complete date
void DS_RTC_SET_CURRENT_DATE(DS_RTC_Config_t* pRTC, I2C_Handle_t* pI2C);

//Gets the current time
void DS_RTC_GET_CURRENT_TIME(DS_RTC_Config_t* pRTC, I2C_Handle_t* pI2C);

//Gets the current complete date
void DS_RTC_GET_CURRENT_DATE(DS_RTC_Config_t* pRTC, I2C_Handle_t* pI2C);

//Gets the current complete date and time
void DS_RTC_GET_CURRENT_TIME_DATE(DS_RTC_Config_t* pRTC, I2C_Handle_t* pI2C, uint8_t* pRxbuffer);

#endif /* INC_DS3231_H_ */
