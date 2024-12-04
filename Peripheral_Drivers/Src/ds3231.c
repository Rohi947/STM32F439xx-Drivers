/*
 * ds3231.c
 *
 *  Created on: Nov 21, 2024
 *      Author: mrohi
 */


#include "ds3231.h"

void DS_RTC_INIT(DS_RTC_Config_t* pRTC)
{
	//Convert Decimals/Binary into BCD

	/**********************************************************************************************************************
	 ********************************************** TIME INIT ************************************************************
	 *********************************************************************************************************************/

	//Seconds
	pRTC->Seconds 	= (((pRTC->Seconds	/10) <<4) | ((pRTC->Seconds	% 10) & 0xf));

	//Minutes
	pRTC->Minutes   = (((pRTC->Minutes	/10) <<4) | ((pRTC->Minutes	% 10) & 0xf));

	//Hours
	if(pRTC->Hours_12_24 == DS_RTC_HOURS_12)
	{
		//12 Hour Mode

		pRTC->Hours |= (0x1 << DS_RTC_POS_HOURS_12_24);

		pRTC->Hours	 = (((pRTC->Hours /10) <<4) | ((pRTC->Hours % 10) & 0xf));

		if(pRTC->Hours_AM_PM == DS_RTC_HOURS_AM)
		{
			pRTC->Hours &= ~(0x1 << DS_RTC_POS_HOURS_AM_PM_20S);
		}
		else
		{
			pRTC->Hours |= (0x1 << DS_RTC_POS_HOURS_AM_PM_20S);
		}
	}
	else
	{
		//24 Hours mode
		pRTC->Hours &= ~(0x1 << DS_RTC_POS_HOURS_12_24);

		pRTC->Hours	= (((pRTC->Hours /10) <<4) | ((pRTC->Hours % 10) & 0xf));
	}

	//Day
	pRTC->Day    = (pRTC->Day & (0x7));

	//Date
	pRTC->Date   = (((pRTC->Date  /10) <<4) | ((pRTC->Date % 10) & 0xf));

	//Month_Century
	pRTC->Month_Century	    = (((pRTC->Month_Century /10) <<4) | ((pRTC->Month_Century % 10) & 0xf));
	pRTC->Month_Century |= (pRTC->Month_Century_Century << DS_RTC_POS_MONTH_CENTURY);

	//Year
	pRTC->Year   = (((pRTC->Year /10) <<4) | ((pRTC->Year % 10) & 0xf));

	/**********************************************************************************************************************
	 ********************************************** ALM1 INIT ************************************************************
	 *********************************************************************************************************************/

	if(pRTC->ALM1_STATUS)
	{
		//Enable ALM1
		pRTC->Control |= (0x1);

		//Enable Interrupt
		pRTC->Control |= (0x1 << 2);

		//Seconds
		pRTC->ALM1_Seconds 	= (((pRTC->ALM1_Seconds	/10) <<4) | ((pRTC->ALM1_Seconds	% 10) & 0xf));

		//Minutes
		pRTC->ALM1_Minutes   = (((pRTC->ALM1_Minutes /10) <<4) | ((pRTC->ALM1_Minutes	% 10) & 0xf));

		//Hours
		if(pRTC->ALM1_Hours_12_24 == DS_RTC_HOURS_12)
		{
			//12 Hour Mode

			pRTC->ALM1_Hours |= (0x1 << DS_RTC_POS_HOURS_12_24);

			pRTC->ALM1_Hours	 = (((pRTC->ALM1_Hours /10) <<4) | ((pRTC->ALM1_Hours % 10) & 0xf));

			if(pRTC->ALM1_Hours_AM_PM == DS_RTC_HOURS_AM)
			{
				pRTC->ALM1_Hours &= ~(0x1 << DS_RTC_POS_HOURS_AM_PM_20S);
			}
			else
			{
				pRTC->ALM1_Hours |= (0x1 << DS_RTC_POS_HOURS_AM_PM_20S);
			}
		}
		else
		{
			//24 Hours mode
			pRTC->ALM1_Hours &= ~(0x1 << DS_RTC_POS_HOURS_12_24);

			pRTC->ALM1_Hours	= (((pRTC->ALM1_Hours /10) <<4) | ((pRTC->ALM1_Hours % 10) & 0xf));
		}

		//Days & Date
		if(pRTC->ALM1_DY_DT == 1)
		{
			//Day
			pRTC->ALM1_Day_Date  |= (0x1 << (DS_RTC_POS_ALM1_DAY_DATE_DY_DT));
		}
		else
		{
			//Date
			pRTC->ALM1_Day_Date   |= (((pRTC->ALM1_Day_Date  /10) <<4) | ((pRTC->ALM1_Day_Date % 10) & 0xf));
			pRTC->ALM1_Day_Date  &= ~(0x1 << (DS_RTC_POS_ALM2_DAY_DATE_DY_DT));

		}

		//Alarm Condition

		//A1M1
		pRTC->ALM1_Seconds |= ((pRTC->ALM1_RATE & (0x1)) << DS_RTC_POS_ALM1_SEC_A1M1);

		//A1M2
		pRTC->ALM1_Minutes |= ((pRTC->ALM1_RATE>>1) & (0x1) << DS_RTC_POS_ALM1_MIN_A1M2);

		//A1M3
		pRTC->ALM1_Hours |= ((pRTC->ALM1_RATE>>2) & (0x1) << DS_RTC_POS_ALM1_HOUR_A1M3);

		//A1M1
		pRTC->ALM1_Day_Date |= (((pRTC->ALM1_RATE>>3) & (0x1)) << DS_RTC_POS_ALM1_DAY_DATE_A1M4);

	}
	else
	{
		//Disable ALM1
		pRTC->Control &= ~(0x1 << 1);
	}

	/**********************************************************************************************************************
	 ********************************************** ALM2 INIT ************************************************************
	 *********************************************************************************************************************/

	if(pRTC->ALM2_STATUS)
	{
		//Enable ALM2
		pRTC->Control |= (0x1 << 1);

		//Enable Interrupt
		pRTC->Control |= (0x1 << 2);

		//Minutes
		pRTC->ALM2_Minutes   = (((pRTC->ALM2_Minutes /10) <<4) | ((pRTC->ALM2_Minutes	% 10) & 0xf));

		//Hours
		if(pRTC->ALM2_Hours_12_24 == DS_RTC_HOURS_12)
		{
			//12 Hour Mode

			pRTC->ALM2_Hours |= (0x1 << DS_RTC_POS_HOURS_12_24);

			pRTC->ALM2_Hours	 = (((pRTC->ALM2_Hours /10) <<4) | ((pRTC->ALM2_Hours % 10) & 0xf));

			if(pRTC->ALM2_Hours_AM_PM == DS_RTC_HOURS_AM)
			{
				pRTC->ALM2_Hours &= ~(0x1 << DS_RTC_POS_HOURS_AM_PM_20S);
			}
			else
			{
				pRTC->ALM2_Hours |= (0x1 << DS_RTC_POS_HOURS_AM_PM_20S);
			}
		}
		else
		{
			//24 Hours mode
			pRTC->ALM2_Hours &= ~(0x1 << DS_RTC_POS_HOURS_12_24);

			pRTC->ALM2_Hours	= (((pRTC->ALM2_Hours /10) <<4) | ((pRTC->ALM2_Hours % 10) & 0xf));
		}

		//Days & Date
		if(pRTC->ALM2_DY_DT == 1)
		{
			//Day
			pRTC->ALM2_Day_Date  |= (0x1 << (DS_RTC_POS_ALM2_DAY_DATE_DY_DT));
		}
		else
		{
			//Date
			pRTC->ALM2_Day_Date   |= (((pRTC->ALM2_Day_Date  /10) <<4) | ((pRTC->ALM2_Day_Date % 10) & 0xf));
			pRTC->ALM2_Day_Date  &= ~(0x1 << (DS_RTC_POS_ALM2_DAY_DATE_DY_DT));

		}

		//Alarm Condition

		//A2M2
		pRTC->ALM2_Minutes |= ((pRTC->ALM2_RATE & (0x1)) << DS_RTC_POS_ALM2_MIN_A2M2);

		//A2M3
		pRTC->ALM2_Hours |= ((pRTC->ALM2_RATE>>1) & (0x1) << DS_RTC_POS_ALM2_HOUR_A2M3);

		//A2M4
		pRTC->ALM2_Day_Date |= ((pRTC->ALM2_RATE>>2) & (0x1) << DS_RTC_POS_ALM2_DAY_DATE_A2M4);
	}
	else
	{
		//Disable ALM2
		pRTC->Control &= ~(0x1 << 1);

		if(pRTC->ALM1_STATUS == DISABLE)
		{
			//Disable Interrupt
			pRTC->Control &= ~(0x1 << 2);
		}
	}
}

//Configures DS_RTC as a stopwatch
void DS_RTC_CONFIG_STOPWATCH(DS_RTC_Config_t* pRTC, I2C_Handle_t* pI2C)
{
	//Set Up DS_RTC as stop watch.
	//DS3231 starts ticking as soon the register is written to.
	//Just Write 00 to the timing registers.

	//Serialize struct data
	uint8_t serialized_data[17] = {	DS_REG_BASE_ADDR,
									pRTC->Seconds,
									pRTC->Minutes,
									pRTC->Hours,
									pRTC->Day,
									pRTC->Date,
									pRTC->Month_Century,
									pRTC->Year,
									pRTC->ALM1_Seconds,
									pRTC->ALM1_Minutes,
									pRTC->ALM1_Hours,
									pRTC->ALM1_Day_Date,
									pRTC->ALM2_Minutes,
									pRTC->ALM2_Hours,
									pRTC->ALM2_Day_Date,
									pRTC->Control,
									pRTC->Cntrl_status
								 };

	I2C_MasterSendData(pI2C, serialized_data, 17, DS3231_ADDRESS);
}

//Sets the DS_RTC to current time
void DS_RTC_SET_CURRENT_TIME(DS_RTC_Config_t* pRTC, I2C_Handle_t* pI2C);

//Sets the DS_RTC to current complete date
void DS_RTC_SET_CURRENT_DATE(DS_RTC_Config_t* pRTC, I2C_Handle_t* pI2C);

//Gets the current time
void DS_RTC_GET_CURRENT_TIME(DS_RTC_Config_t* pRTC, I2C_Handle_t* pI2C);

//Gets the current complete date
void DS_RTC_GET_CURRENT_DATE(DS_RTC_Config_t* pRTC, I2C_Handle_t* pI2C);

//Gets the current complete date and time
void DS_RTC_GET_CURRENT_TIME_DATE(DS_RTC_Config_t* pRTC, I2C_Handle_t* pI2C, uint8_t* pRxbuffer)
{
	//Read contents of all registers from the ds3231
	//First Initialize the register pointer to point to seconds
	uint8_t tempreg = DS_REG_Seconds;
	I2C_MasterSendData(pI2C, &tempreg, 1, DS3231_ADDRESS);

	pI2C->Sr = 0;

	//And get the current time and date
	I2C_MasterReceiveData(pI2C, pRxbuffer, 3, DS3231_ADDRESS);

	//Convert BCD into Decimal/Binary
	pRxbuffer[0] 	= (((pRxbuffer[0] >> 4)	*10) + (pRxbuffer[0] & 0xf));
	pRxbuffer[1] 	= (((pRxbuffer[1] >> 4)	*10) + (pRxbuffer[1] & 0xf));

	//Hours
	if(pRTC->Hours_12_24 == DS_RTC_HOURS_24)
	{
		pRxbuffer[2]  	= ((((pRxbuffer[2] >> 4) & 0x3) *10) + (pRxbuffer[2] & 0xf));
	}
	else
	{
		pRxbuffer[2]  	= ((((pRxbuffer[2] >> 4) & 0x1) *10) + (pRxbuffer[2] & 0xf));
	}
	pRxbuffer[4]	= (((pRxbuffer[4] >> 4)	*10) + (pRxbuffer[4] & 0xf));
	pRxbuffer[5] 	= (((pRxbuffer[5] >> 4)	*10) + (pRxbuffer[5] & 0xf));
	pRxbuffer[6] 	= (((pRxbuffer[6] >> 4)	*10) + (pRxbuffer[6] & 0xf));
}
