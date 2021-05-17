/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               VS1003.c
** Descriptions:            The VS1003 application function
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2011-2-27
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "vs1003.h"

static uint8_t VS1003_SPI_ReadByte(void)
{
// 	HAL_StatusTypeDef res;
// 	uint8_t rxData[1] = {0, };

// 	res = HAL_SPI_Receive(&hspi3, rxData, sizeof(rxData), HAL_MAX_DELAY);
// 	if (res != HAL_OK);
// 		//printf("HAL_SPI_Receive Error\r\n");

//   return rxData[0];

//first clear over run flag (it can occur because we set up spi in duplex-mode, but just trans or receive once)
/*
An overrun condition occurs when the master device has sent data bytes and the slave
device has not cleared the RXNE bit resulting from the previous data byte transmitted.
When an overrun condition occurs:
• the OVR bit is set and an interrupt is generated if the ERRIE bit is set.
In this case, the receiver buffer contents will not be updated with the newly received data
from the master device. A read from the SPI_DR register returns this byte. All other
subsequently transmitted bytes are lost.
Clearing the OVR bit is done by a read from the SPI_DR register followed by a read access
to the SPI_SR register.
*/
//LL_SPI_IsActiveFlag_BSY(SPI3);
	if(LL_SPI_IsActiveFlag_OVR(SPI3))
	{
		volatile uint32_t tmpreg_ovr = 0x00U;
		tmpreg_ovr = SPI3->DR;
		tmpreg_ovr = SPI3->SR;
		(void)tmpreg_ovr;
	}

	uint8_t rxData = 0x00;
	//trans dump data before read
	//wait tx is done (when done TXE = 1)
	while(!LL_SPI_IsActiveFlag_TXE(SPI3));
	SPI3->DR = rxData;

	//wait until rxne = 1
	while(!LL_SPI_IsActiveFlag_RXNE(SPI3));
	rxData = SPI3->DR;

	return rxData;
}

static void VS1003_SPI_WriteByte( uint8_t TxData )
{
// 	HAL_StatusTypeDef res;
// 	uint8_t cmd[1] = {0, };

// 	cmd[0] = TxData;
// 	res = HAL_SPI_Transmit(&hspi3, cmd, sizeof(cmd), HAL_MAX_DELAY);
// 	if (res != HAL_OK);
// 		//printf("HAL_SPI_Transmit Error\r\n");

//   return 0;
	if(LL_SPI_IsActiveFlag_OVR(SPI3))
	{
		volatile uint32_t tmpreg_ovr = 0x00U;
		tmpreg_ovr = SPI3->DR;
		tmpreg_ovr = SPI3->SR;
		(void)tmpreg_ovr;
	}

	//trans data before read
	//wait tx is done (when done TXE = 1) (tx buffer is empty)
	while(!LL_SPI_IsActiveFlag_TXE(SPI3));
	SPI3->DR = TxData;

	//wait until rxne = 1
	while(!LL_SPI_IsActiveFlag_RXNE(SPI3));
	TxData = SPI3->DR; //clear RXNE flag to avoid overrun occur

	return 0;
}

void vs1003_gpio_init()
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);


  /**/
  LL_GPIO_SetOutputPin(GPIOD, XCS_Pin|XDCS_Pin|XRST_Pin);

  /**/
  GPIO_InitStruct.Pin = XCS_Pin|XDCS_Pin|XRST_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE15);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_15;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(DREQ_GPIO_Port, DREQ_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(DREQ_GPIO_Port, DREQ_Pin, LL_GPIO_MODE_INPUT);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),7, 0));
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void vs1003_spi_init()
{
	/* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /**SPI3 GPIO Configuration
  PC10   ------> SPI3_SCK
  PC11   ------> SPI3_MISO
  PC12   ------> SPI3_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* SPI3 DMA Init */

  /* SPI3_TX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_0);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_VERYHIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)&SPI3->DR);

  /* Enable HT & TC interrupts */
  //LL_DMA_EnableIT_HT(DMA2, LL_DMA_STREAM_2);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);

  /* DMA1_Stream5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),7, 0));
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  /* SPI3 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI3, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_EnableDMAReq_TX(SPI3);
  LL_SPI_Enable(SPI3);

	//just enable after set memory addres, length
	//LL_DMA_EnableStream(DMA1, LL_DMA_Stream5);
// 	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)sim_dma_buffer);
//   LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, sim_dma_buffer_size);
}

void VS1003_Init(void)
{
	vs1003_gpio_init();
	vs1003_spi_init();
  MP3_Reset(0);
  HAL_Delay(200);
  MP3_Reset(1);

  MP3_DCS(1);
  MP3_CCS(1);
}

/*******************************************************************************
*******************************************************************************/
void VS1003_WriteReg( uint8_t reg, uint16_t value )
{
	while(  MP3_DREQ == 0 );           /* */

	//VS1003_SPI_SetSpeed( SPI_SPEED_LOW );
	MP3_DCS(1);
	MP3_CCS(0);
	VS1003_SPI_WriteByte(VS_WRITE_COMMAND); /*  VS1003 */
	VS1003_SPI_WriteByte(reg);
	VS1003_SPI_WriteByte(value>>8);
	VS1003_SPI_WriteByte(value);
	MP3_CCS(1);
	MP3_DCS(0);
	//VS1003_SPI_SetSpeed( SPI_SPEED_HIGH );
}

/*******************************************************************************
*******************************************************************************/
uint16_t VS1003_ReadReg( uint8_t reg)
{
	uint16_t value;

	while(  MP3_DREQ == 0 );

	//VS1003_SPI_SetSpeed( SPI_SPEED_LOW );

	MP3_DCS(1);
	MP3_CCS(0);

	VS1003_SPI_WriteByte(VS_READ_COMMAND);/* VS1003 */
	VS1003_SPI_WriteByte( reg );
	value = VS1003_SPI_ReadByte();
	value = value << 8;
	value |= VS1003_SPI_ReadByte();

	MP3_CCS(1);
	MP3_DCS(0);
	//VS1003_SPI_SetSpeed( SPI_SPEED_HIGH );

	return value;
}

/*******************************************************************************
*******************************************************************************/
void VS1003_ResetDecodeTime(void)
{
   VS1003_WriteReg(SPI_DECODE_TIME, 0x0000);
   VS1003_WriteReg(SPI_DECODE_TIME, 0x0000); /* */
}

/*******************************************************************************
*******************************************************************************/
uint16_t VS1003_GetDecodeTime(void)
{
   return VS1003_ReadReg(SPI_DECODE_TIME);
}

/*******************************************************************************
*******************************************************************************/
void VS1003_SoftReset(void)
{
	uint8_t retry;

	while(  MP3_DREQ ==0 );

	VS1003_WriteReg(SPI_MODE, (SM_SDINEW | SM_RESET));  /* */
	HAL_Delay(10);                        /* 1.35ms */

	while(  MP3_DREQ ==0 );

	//check spi read write
	while( VS1003_ReadReg(SPI_CLOCKF) != 0X9800 )
		{
			VS1003_WriteReg(SPI_CLOCKF, 0X9800);
			if( retry++ > 100 )
			{
				//printf("SPI_CLOCKF Set Error\r\n");
				break;
			}
		}

	VS1003_WriteReg(SPI_VOL, 0x4040);
	VS1003_ResetDecodeTime();
}


void VS1003_SetVol(uint16_t volt)
{
   VS1003_WriteReg(SPI_VOL, volt);
}

uint8_t szBeepMP3[] = // 432
{
    0xff, 0xe2, 0x19, 0xc0, 0xd4, 0x80, 0x00, 0x0a, 0x61, 0x76, 0x72, 0xe9, 0x41, 0x30, 0x01, 0x0d,
    0xbe, 0x90, 0xcc, 0x13, 0x0f, 0xc6, 0xe3, 0xf8, 0xdf, 0xfe, 0x05, 0xfc, 0x0b, 0xc0, 0xab, 0xc8,
    0x0b, 0xff, 0xff, 0xf6, 0x8b, 0xbf, 0xe2, 0x23, 0x2c, 0x10, 0x82, 0x18, 0xf7, 0x7a, 0xd7, 0x77,
    0xad, 0x11, 0x8e, 0x61, 0x04, 0x00, 0x3a, 0xf0, 0xff, 0xb0, 0x04, 0xb1, 0x00, 0x00, 0x06, 0x59,
    0xc3, 0x99, 0x00, 0x00, 0x70, 0x0b, 0x80, 0x00, 0xff, 0xe2, 0x19, 0xc0, 0xc6, 0xe8, 0x07, 0x0b,
		0x11, 0x9e, 0xee, 0xf9, 0x81, 0x68, 0x02, 0x01, 0xb8, 0x00, 0x00, 0x39, 0x42, 0x12, 0xff, 0xff,
		0x70, 0x0f, 0xff, 0xae, 0xbf, 0xab, 0xfe, 0xa4, 0x1b, 0xf0, 0xe5, 0x37, 0xd6, 0x1d, 0x7e, 0xa6,
    0x7f, 0xe3, 0x30, 0xdf, 0xfe, 0x33, 0x0e, 0xbc, 0xb1, 0x97, 0xf5, 0x07, 0x7b, 0x27, 0xff, 0xff,
    0xff, 0x25, 0x5d, 0xb8, 0xce, 0x9b, 0x0a, 0x7a, 0x9b, 0x96, 0x81, 0xaf, 0x92, 0x02, 0x83, 0x97,
    0xff, 0xe2, 0x19, 0xc0, 0x06, 0x63, 0x13, 0x0b, 0x79, 0x7e, 0x90, 0x21, 0xc0, 0xd8, 0x00, 0xb4,
    0xa6, 0xd4, 0xa6, 0x97, 0x1f, 0xff, 0xfe, 0x63, 0x84, 0xa9, 0x4a, 0x93, 0xe8, 0xaa, 0xe0, 0x7a,
    0xa0, 0xe5, 0xaa, 0x4e, 0xa6, 0xb2, 0xea, 0xbc, 0x77, 0xf5, 0x00, 0xdd, 0xb0, 0x18, 0x03, 0xff,
    0xf5, 0x90, 0x1e, 0x72, 0x2e, 0x6f, 0xff, 0xfe, 0x7c, 0xc7, 0xff, 0xa0, 0x81, 0x4c, 0x52, 0x60,
    0x64, 0x4f, 0x09, 0x88, 0xcd, 0x93, 0xe6, 0xff, 0xff, 0xe2, 0x19, 0xc0, 0xcd, 0x5a, 0x1e, 0x0b,
    0x69, 0x76, 0xba, 0xe0, 0x08, 0x68, 0x6c, 0xf9, 0x99, 0xba, 0x41, 0xfa, 0x00, 0x61, 0x80, 0x2d,
    0xe8, 0xa0, 0x33, 0x05, 0x77, 0x35, 0x4f, 0x1b, 0x5b, 0x38, 0x00, 0x07, 0x1f, 0xf9, 0x85, 0x7f,
    0xcc, 0x3f, 0x3f, 0x0a, 0xf9, 0xaf, 0xf8, 0x43, 0xff, 0xff, 0x35, 0xd6, 0xe1, 0x2b, 0x8d, 0x21,
    0x39, 0x00, 0x64, 0x69, 0x05, 0x74, 0xf0, 0x77, 0x9d, 0x5b, 0x7f, 0xe2, 0xdf, 0x2c, 0x25, 0xf4,
    0xff, 0xe2, 0x19, 0xc0, 0x22, 0x06, 0x29, 0x0f, 0x09, 0x7a, 0xa2, 0x38, 0x08, 0x5e, 0x6e, 0xe8,
    0x00, 0x3c, 0x2d, 0x60, 0xe5, 0x3c, 0x71, 0x77, 0xba, 0x12, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x43,
    0xdf, 0x0d, 0x5f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xe2, 0x19, 0xc0, 0xbc, 0xd1, 0x25, 0x00,
    0x00, 0x02, 0x5c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
//24B mono, mpeg 2, 8kbps
uint8_t mute1framse[] = {0xFF, 0xF3, 0x14, 0xC4, 0x16, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41};
//1200B
uint8_t mute50framse[] = {0xFF, 0xF3, 0x14, 0xC4, 0x16, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0x21, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0x2C, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0x37, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0x42, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0x4D, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0x58, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0x63, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0x6E, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0x79, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0x84, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0x8F, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0x9A, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0xA5, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0xB0, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0xBB, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0xC6, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0xD1, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0xDC, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0xE7, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0xF2, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x4C, 0x41, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x4D, 0x45, 0x33, 0x2E, 0x31, 0x30, 0x30, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xFF, 0xF3, 0x14, 0xC4, 0xF4, 0x0, 0x0, 0x3, 0x48, 0x0, 0x0, 0x0, 0x0, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};

void VS1003_PlayBeep(void)
{
	uint32_t in;

	MP3_CCS(1);
	MP3_DCS(0);

	for (in=0; in<sizeof(szBeepMP3); in++)
		VS1003_SPI_WriteByte(szBeepMP3[in]);

	MP3_CCS(0);
	MP3_DCS(1);
//HAL_SPI_Transmit_DMA(&hspi3, packet_receiver[index_packet_read].frame_array, packet_receiver[index_packet_read].length);
	//for(i=0; i<30;i++)
	//	VS1003_SPI_WriteByte(0x00);
}

void VS1003_PlayBeep_DMA(void)
{
	MP3_CCS(1);
    MP3_DCS(0);
	//HAL_SPI_Transmit_DMA(&hspi3, szBeepMP3, sizeof(szBeepMP3));
}

void VS1003_PlayMute_DMA(void)
{
	MP3_CCS(1);
    MP3_DCS(0);
	//HAL_SPI_Transmit_DMA(&hspi3, mute50framse, 24);
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

