/*
 * stm32f439xx_usart_driver.h
 *
 *  Created on: Nov 19, 2024
 *      Author: mrohi
 */

#ifndef INC_STM32F439XX_USART_DRIVER_H_
#define INC_STM32F439XX_USART_DRIVER_H_

#include "stm32f439xx.h"

typedef struct{
	uint32_t Mode;
	uint32_t Word_Length;
	uint32_t Stop_Bits;
	uint32_t Baud;
	uint32_t Parity_Control;
	uint32_t HWControl;
}USART_Config_t;

typedef struct{
	USART_RegDef_t* pUSART;
	USART_Config_t USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint8_t DevAddr;
	uint32_t RxSize;
	uint8_t Sr;
}USART_Handle_t;

//Macros

#define MODE_TX																0
#define MODE_RX																1
#define MODE_TX_RX															2

#define OVERSAMP_16															0
#define OVERSAMP_8															1

#define USART_DISABLE														0
#define USART_ENABLE														1

#define WORD_LENGTH_8														0
#define WORD_LENGTH_9														1

#define PARITY_CONTROL_DISABLED												0
#define PARITY_CONTROL_EN_ODD												1
#define PARITY_CONTROL_EN_EVEN												2

#define PARITY_EVEN															0
#define PARITY_ODD															1

#define TE_DISABLE															0
#define TE_ENABLE															1

#define RE_DISABLE															0
#define RE_ENABLE															1

#define STOP_1																0
#define STOP_0_5															1
#define STOP_2																2
#define STOP_1_5															3


//BRR
#define BAUD_1200															1200
#define BAUD_2400															2400
#define BAUD_9600															9600
#define BAUD_19200															19200
#define BAUD_38400															38400
#define BAUD_57600															57600
#define BAUD_115200															115200
#define BAUD_230400															230400
#define BAUD_460800															460800
#define BAUD_921600															921600
#define BAUD_2000000														2000000
#define BAUD_3000000														3000000


//FLAGS
#define USART_FLAG_PE														0
#define USART_FLAG_FE														1
#define USART_FLAG_NF														2
#define USART_FLAG_ORE														3
#define USART_FLAG_IDLE														4
#define USART_FLAG_RXNE														5
#define USART_FLAG_TC														6
#define USART_FLAG_TXE														7
#define USART_FLAG_LBD														8
#define USART_FLAG_CTS														9

//Register Bit Positions

//CR1
#define USART_CR1_SBK														0
#define USART_CR1_RWU														1
#define USART_CR1_RE														2
#define USART_CR1_TE														3
#define USART_CR1_IDLEIE													4
#define USART_CR1_RXNEIE													5
#define USART_CR1_TCIE														6
#define USART_CR1_TXEIE														7
#define USART_CR1_PEIE														8
#define USART_CR1_PS														9
#define USART_CR1_PCE														10
#define USART_CR1_WAKE														11
#define USART_CR1_M															12
#define USART_CR1_UE														13
#define USART_CR1_OVER8														15

//CR2
#define USART_CR2_ADD														0
#define USART_CR2_LBDL														5
#define USART_CR2_LBDIE														6
#define USART_CR2_LBCL														8
#define USART_CR2_CPHA														9
#define USART_CR2_CPOL														10
#define USART_CR2_CLKEN														11
#define USART_CR2_STOP														12
#define USART_CR2_LINEN														14

//CR3
#define USART_CR3_EIE														0
#define USART_CR3_IREN														1
#define USART_CR3_IRLP														2
#define USART_CR3_HDSEL														3
#define USART_CR3_NACK														4
#define USART_CR3_SCEN														5
#define USART_CR3_DMAR														6
#define USART_CR3_DMAT														7
#define USART_CR3_RTSE														8
#define USART_CR3_CTSE														9
#define USART_CR3_CTSIE														10
#define USART_CR3_ONEBIT													11

//BRR
#define DIV_FRACTION														0
#define DIV_MANTISSA														4

#define USART_HW_FLOW_CTRL_CTS												0
#define USART_HW_FLOW_CTRL_RTS												1
#define USART_HW_FLOW_CTRL_CTS_RTS											2
#define USART_HW_FLOW_NO_CTRL												3

#define USART_READY															0
#define USART_BUSY_IN_RX													1
#define USART_BUSY_IN_TX													2


#define USART_EVENT_TX_CMPLT												0
#define USART_EVENT_RX_CMPLT												1
#define	USART_EVENT_IDLE      												2
#define	USART_EVENT_CTS       												3
#define	USART_EVENT_PE        												4
#define	USART_ERR_FE     													5
#define	USART_ERR_NE    	 												6
#define	USART_ERR_ORE    													7

/******************************************************************************************************************************************************
 * 												API's SUPPORTED BY THIS DRIVER
 *****************************************************************************************************************************************************/

//USART Clock Setup
void USART_ClockControl(USART_RegDef_t *pUSART, uint8_t EnorDi);

//USART Init and DeInit
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSART);

//Blocking APIs

//USART Receive
void USART_ReceiveData(USART_Handle_t *pUSART, uint8_t *pRxBuffer, uint8_t len, uint8_t Slave_Address);

//USART Transmit
void USART_SendData(USART_Handle_t *pUSART, uint8_t *pTxBuffer, uint8_t len, uint8_t Slave_Address);


//Interrupt Based

//USART Receive
uint8_t USART_ReceiveData_IT(USART_Handle_t *pUSART, uint8_t *pRxBuffer, uint8_t len);

//USART Transmit
uint8_t USART_SendData_IT(USART_Handle_t *pUSART, uint8_t *pTxBuffer, uint8_t len);


//IRQ
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSART);
void USART_HandleTXEInterrupt(USART_Handle_t *pUSART);
void USART_HandleRXNEInterrupt(USART_Handle_t *pUSART);

//Other Peripheral Control APIs
void USART_PeripheralControl(USART_RegDef_t *pUSART, uint8_t EnorDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSART, uint32_t FlagName);
void USART_CloseSendData(USART_Handle_t *pUSARTHandle);
void USART_CloseReceiveData(USART_Handle_t *pUSARTHandle);

//Application Callback
void USART_ApplicationEventCallback(USART_Handle_t *pUSART, uint8_t AppEvent);




#endif /* INC_STM32F439XX_USART_DRIVER_H_ */
