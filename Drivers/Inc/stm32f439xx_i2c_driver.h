/*
 * stm32f439xx_i2c_driver.h
 *
 *  Created on: Nov 6, 2024
 *      Author: mrohi
 */

#ifndef INC_STM32F439XX_I2C_DRIVER_H_
#define INC_STM32F439XX_I2C_DRIVER_H_

#include "stm32f439xx.h"

//I2C Config

typedef struct{
	uint8_t Device_Address;
	uint32_t SCL_Speed;
	uint8_t Duty_Cycle;
	uint8_t ACK_Control;
}I2C_Config_t;

//I2C Handle
typedef struct{
	I2C_RegDef_t* 	pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer;
	uint8_t 		*pRxBuffer;
	uint32_t 		TxLen;
	uint32_t		RxLen;
	uint8_t			TxRxState;
	uint8_t			DevAddr;
	uint32_t		RxSize;
	uint8_t			Sr;
	uint8_t			mode;
}I2C_Handle_t;

//I2C Start
#define I2C_START												1

//I2C Stop
#define I2C_STOP												1

//I2C Modes
#define I2C_Mode_Slave											0
#define I2C_Mode_Master											1

//I2C Speed
#define I2C_SCL_SPEED_SM										100000U
#define I2C_SCL_SPEED_FM_2K										200000U
#define I2C_SCL_SPEED_FM_4K										400000U

//I2C DUTY
#define I2C_FM_DUTY_2											0
#define I2C_FM_DUTY_16_9										1

//ACK Control
#define I2C_NO_ACK												0
#define I2C_ACK													1

//CR1 Register Bit Positions
#define I2C_CR1_PE												0
#define I2C_CR1_SMBUS											1
#define I2C_CR1_SMB_TYPE										3
#define I2C_CR1_ENARP											4
#define I2C_CR1_ENPEC											5
#define I2C_CR1_ENGC											6
#define I2C_CR1_NO_STRETCH										7
#define I2C_CR1_START											8
#define I2C_CR1_STOP											9
#define I2C_CR1_ACK												10
#define I2C_CR1_POS												11
#define I2C_CR1_PEC												12
#define I2C_CR1_ALERT											13
#define I2C_CR1_SWRST											15

//CR2 Register Bit Positions
#define I2C_CR2_FREQ											0
#define I2C_CR2_RESERVED
#define I2C_CR2_ITERREN											8
#define I2C_CR2_ITEVFEN											9
#define I2C_CR2_ITBUFEN											10
#define I2C_CR2_DMAEN											11
#define I2C_CR2_LAST											12

//SR1 Register Bit Positions
#define I2C_SR1_SB												0
#define I2C_SR1_ADDR											1
#define I2C_SR1_BTF												2
#define I2C_SR1_ADD10											3
#define I2C_SR1_STOPF											4
#define I2C_SR1_RxNE											6
#define I2C_SR1_TxE												7
#define I2C_SR1_BERR											8
#define I2C_SR1_ARLO											9
#define I2C_SR1_AF												10
#define I2C_SR1_OVR												11
#define I2C_SR1_PEC_ERR											12
#define I2C_SR1_TIME_OUT										14
#define I2C_SR1_SMB_ALERT										15

//SR2 Register Bit Positions
#define I2C_SR2_MSL												0
#define I2C_SR2_BUSY											1
#define I2C_SR2_TRA												2
#define I2C_SR2_RESERVED										3
#define I2C_SR2_GEN_CALL										4
#define I2C_SR2_SMBDE_FAULT										5
#define I2C_SR2_SMB_HOST										6
#define I2C_SR2_DUALF											7
#define I2C_SR2_PEC												8

//OAR1 Register Bit Positions
#define I2C_OAR1_ADD_10_BIT										0
#define I2C_OAR1_ADD_7_BIT										1
#define I2C_OAR1_ADD_EX_ADDRESS									8
#define I2C_OAR1_ADD_MODE										15

//OAR2 Register Bit Positions
#define I2C_OAR2_ENDUAL											0
#define I2C_OAR2_ADD2											1

//CCR Register Bit Positions
#define I2C_CCR_CCR												0
#define I2C_CCR_DUTY											14
#define I2C_CCR_Speed_MODE										15

//TRISE Register Bit Positions
#define I2C_TRISE												0

//FLTR Register Bit Positions
#define I2C_FLTR_DNF											0
#define I2C_FLTR_ANOFF											4

//FREQ
#define FREQ_2MHz												2
#define FREQ_4MHz												4
#define FREQ_5MHz												5
#define FREQ_10MHz												10
#define FREQ_15MHz												15
#define FREQ_20MHz												20
#define FREQ_25MHz												25
#define FREQ_30MHz												30
#define FREQ_35MHz												35
#define FREQ_40MHz												40
#define FREQ_45MHz												45
#define FREQ_50MHz												50

//I2C Flags
#define I2C_FLAG_SR1_SB												0
#define I2C_FLAG_SR1_ADDR											1
#define I2C_FLAG_SR1_BTF											2
#define I2C_FLAG_SR1_ADD10											3
#define I2C_FLAG_SR1_STOPF											4
#define I2C_FLAG_SR1_RxNE											6
#define I2C_FLAG_SR1_TxE											7
#define I2C_FLAG_SR1_BERR											8
#define I2C_FLAG_SR1_ARLO											9
#define I2C_FLAG_SR1_AF												10
#define I2C_FLAG_SR1_OVR											11
#define I2C_FLAG_SR1_PEC_ERR										12
#define I2C_FLAG_SR1_TIME_OUT										14
#define I2C_FLAG_SR1_SMB_ALERT										15

#define I2C_FLAG_SR2_MSL											16
#define I2C_FLAG_SR2_BUSY											17
#define I2C_FLAG_SR2_TRA											18
#define I2C_FLAG_SR2_GEN_CALL										20
#define I2C_FLAG_SR2_SMBDE_FAULT									21
#define I2C_FLAG_SR2_SMB_HOST										22
#define I2C_FLAG_SR2_DUALF											23


//I2C Application Status

#define I2C_READY													0
#define I2C_BUSY_IN_RX												1
#define I2C_BUSY_IN_TX												2

//I2C Event
#define I2C_EV_TX_CMPLT												0
#define I2C_EV_RX_CMPLT												1
#define I2C_EV_STOP													2
#define I2C_EV_DATA_REQ												3
#define I2C_EV_DATA_RCV												4


/******************************************************************************************************************************************************
 * 												API's SUPPORTED BY THIS DRIVER
 *****************************************************************************************************************************************************/

//SPI Clock Setup
void I2C_ClockControl(I2C_RegDef_t *pI2C, uint8_t EnorDi);

//SPI Init and DeInit
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2C);

//Blocking APIs

//I2C Receive
void I2C_MasterReceiveData(I2C_Handle_t *pI2C, uint8_t *pRxBuffer, uint8_t len, uint8_t Slave_Address);

//SPI Transmit
void I2C_MasterSendData(I2C_Handle_t *pI2C, uint8_t *pTxBuffer, uint8_t len, uint8_t Slave_Address);


//Interrupt Based

//I2C Receive
uint8_t I2C_MasterReceiveData_IT(I2C_Handle_t *pI2C, uint8_t *pRxBuffer, uint8_t len, uint8_t Slave_Address);

//I2C Transmit
uint8_t I2C_MasterSendData_IT(I2C_Handle_t *pI2C, uint8_t *pTxBuffer, uint8_t len, uint8_t Slave_Address);


//IRQ
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C1_EV_IRQHandling(I2C_Handle_t *pI2C);
void I2C1_EV_IRQHandling(I2C_Handle_t *pI2C);
void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2C);
void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2C);

//Other Peripheral Control APIs
void I2C_PeripheralControl(I2C_RegDef_t *pI2C, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

//Application Callback
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C, uint8_t AppEvent);

#endif /* INC_STM32F439XX_I2C_DRIVER_H_ */
