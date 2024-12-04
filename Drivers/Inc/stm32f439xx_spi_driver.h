#ifndef INC_STM32F439XX_SPI_DRIVER_H_
#define INC_STM32F439XX_SPI_DRIVER_H_

#include "stm32f439xx.h"

//SPI Config
typedef struct{
	uint8_t Mode;
	uint8_t Bus_config;
	uint8_t DFF;
	uint8_t CPHA;
	uint8_t CPOL;
	uint8_t SSM;
	uint8_t Speed;
	uint8_t SSI;
}SPI_Config_t;

//SPI Handle
typedef struct{
	SPI_RegDef_t* pSPIx;
	SPI_Config_t SPI_Config;

	//For Interrupt
	uint8_t 	*pTxBuffer;
	uint8_t		*pRxBuffer;
	uint8_t		TxLen;
	uint8_t		RxLen;
	uint8_t		TxState;
	uint8_t		RxState;
}SPI_Handle_t;

//CR1 Register Bit Positions
#define SPI_CR1_CPHA																0
#define SPI_CR1_CPOL																1
#define SPI_CR1_MSTR																2
#define SPI_CR1_BR																	3
#define SPI_CR1_SPE																	6
#define SPI_CR1_LSB_FIRST															7
#define SPI_CR1_SSI 																8
#define SPI_CR1_SSM 																9
#define SPI_CR1_RX_ONLY																10
#define SPI_CR1_DFF																	11
#define SPI_CR1_CRC_NEXT															12
#define SPI_CR1_CRC_EN																13
#define SPI_CR1_BIDIOE																14
#define SPI_CR1_BIDIMODE															15

//CR2 Register Bit Positions
#define SPI_CR2_RXDMAEN																0
#define SPI_CR2_TXDMAEN																1
#define SPI_CR2_SSOE																2
#define SPI_CR2_FRF																	4
#define SPI_CR2_ERRIE																5
#define SPI_CR2_RXNEIE																6
#define SPI_CR2_TXEIE																7

//SR Register Bit Positions
#define SPI_SR_RXNE																	0
#define SPI_SR_TXE																	1
#define SPI_SR_CHSIDE																2
#define SPI_SR_UDR																	3
#define SPI_SR_CRCERR																4
#define SPI_SR_MODF																	5
#define SPI_SR_OVR																	6
#define SPI_SR_BSY																	7
#define SPI_SR_FRE																	8


//SPI_Modes
#define SPI_Mode_Slave															0
#define SPI_Mode_Master															1

//SPI Bus Config
#define SPI_Bus_Full_Duplex														0
#define SPI_Bus_Half_Duplex														1
#define SPI_Bus_Simplex_Receive													2

//SPI DFF Config
#define SPI_DFF_8																0
#define SPI_DFF_16																1

//SPI CPHA Config
#define SPI_CPHA_First															0
#define SPI_CPHA_Second															1

//SPI CPOL Config
#define SPI_CPOL_CLK_LOW														0
#define SPI_CPOL_CLK_HIGH														1

//SPI SSM Config
#define SPI_SSM_DI																0
#define SPI_SSM_EN																1

//SPI SSI Config
#define SPI_SSI_DI																0
#define SPI_SSI_EN																1

//SPI Baudrate Values
#define SPI_BAUD_DIV_2															0
#define SPI_BAUD_DIV_4															1
#define SPI_BAUD_DIV_8															2
#define SPI_BAUD_DIV_16															3
#define SPI_BAUD_DIV_32															4
#define SPI_BAUD_DIV_64															5
#define SPI_BAUD_DIV_128														6
#define SPI_BAUD_DIV_256														7

//SPI Flag Macros
#define FLAG_SET																ENABLE
#define FLAG_RESET																DISABLE
#define SPI_FLAG_RXNE															(1 << SPI_SR_RXNE)
#define SPI_FLAG_TXE															(1 << SPI_SR_TXE)
#define SPI_FLAG_CHSIDE															(1 << SPI_SR_CHSIDE)
#define SPI_FLAG_UDR															(1 << SPI_SR_UDR)
#define SPI_FLAG_CRCERR															(1 << SPI_SR_CRCERR)
#define SPI_FLAG_MODF															(1 << SPI_SR_MODF)
#define SPI_FLAG_OVR															(1 << SPI_SR_OVR)
#define SPI_FLAG_BSY															(1 << SPI_SR_BSY)
#define SPI_FLAG_FRE															(1 << SPI_SR_FRE)

//SPI Bus Status
#define SPI_READY																0
#define SPI_BUSY_IN_RX															1
#define SPI_BUSY_IN_TX															2

//Application Callback Event Flags
#define SPI_EVENT_TX_CMPLT														1
#define SPI_EVENT_RX_CMPLT														2
#define SPI_EVENT_OVR_ERR														3


/******************************************************************************************************************************************************
 * 												API's SUPPORTED BY THIS DRIVER
 *****************************************************************************************************************************************************/

//SPI Clock Setup
void SPI_ClockControl(SPI_RegDef_t *pSPI, uint8_t EnorDi);

//SPI Init and DeInit
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPI);

//Blocking APIs

//SPI Receive
void SPI_Receive(SPI_RegDef_t *pSPI, uint8_t *pRxBuffer, uint8_t len);

//SPI Transmit
void SPI_SendData(SPI_RegDef_t *pSPI, uint8_t *pTxBuffer, uint8_t len);

//Interrupt Based

//SPI Receive
uint8_t SPI_Receive_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint8_t len);

//SPI Transmit
uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint8_t len);


//IRQ
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

//Other Peripheral Control APIs
void SPI_PeripheralControl(SPI_RegDef_t *pSPI, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPI, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

//Application Callback

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);

#endif /* INC_STM32F439XX_SPI_DRIVER_H_ */
