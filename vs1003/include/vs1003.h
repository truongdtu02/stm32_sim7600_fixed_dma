/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               VS1003.h
** Descriptions:            The VS1003 application function
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2011-2-17
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

#ifndef _VS1003_H_
#define _VS1003_H_
												   
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
//#include "cmsis_os.h"

/* Private define ------------------------------------------------------------*/
#define SPI_SPEED_HIGH    1  
#define SPI_SPEED_LOW     0

#define DREQ_Pin GPIO_PIN_15
#define DREQ_GPIO_Port GPIOB
#define DREQ_EXTI_IRQn EXTI15_10_IRQn
#define XCS_Pin GPIO_PIN_9
#define XCS_GPIO_Port GPIOD
#define XDCS_Pin GPIO_PIN_11
#define XDCS_GPIO_Port GPIOD
#define XRST_Pin GPIO_PIN_0
#define XRST_GPIO_Port GPIOD

//spi3 stm32f407ve pin
/*
PC10-sck    PC11-miso   PC12-mosi
dreq-PB15
xcs-PD9
xdcs-PD11
xrst-PD0
*/

/* reset */
#define MP3_Reset(x) x ? LL_GPIO_SetOutputPin(XRST_GPIO_Port, XRST_Pin) : LL_GPIO_ResetOutputPin(XRST_GPIO_Port, XRST_Pin)

#define MP3_CCS(x) x ? LL_GPIO_SetOutputPin(XCS_GPIO_Port, XCS_Pin) : LL_GPIO_ResetOutputPin(XCS_GPIO_Port, XCS_Pin)

#define MP3_DCS(x) x ? LL_GPIO_SetOutputPin(XDCS_GPIO_Port, XDCS_Pin) : LL_GPIO_ResetOutputPin(XDCS_GPIO_Port, XDCS_Pin)

#define MP3_DREQ LL_GPIO_ReadInputPort(DREQ_GPIO_Port) >> DREQ_Pin


#define VS_WRITE_COMMAND 	0x02
#define VS_READ_COMMAND 	0x03

/* VS1003 */
#define SPI_MODE        	0x00   
#define SPI_STATUS      	0x01   
#define SPI_BASS        	0x02   
#define SPI_CLOCKF      	0x03   
#define SPI_DECODE_TIME 	0x04   
#define SPI_AUDATA      	0x05   
#define SPI_WRAM        	0x06   
#define SPI_WRAMADDR    	0x07   
#define SPI_HDAT0       	0x08   
#define SPI_HDAT1       	0x09 
#define SPI_AIADDR      	0x0a   
#define SPI_VOL         	0x0b   
#define SPI_AICTRL0     	0x0c   
#define SPI_AICTRL1     	0x0d   
#define SPI_AICTRL2     	0x0e   
#define SPI_AICTRL3     	0x0f 

#define SM_DIFF         	0x01   
#define SM_JUMP         	0x02   
#define SM_RESET        	0x04   
#define SM_OUTOFWAV     	0x08   
#define SM_PDOWN        	0x10   
#define SM_TESTS        	0x20   
#define SM_STREAM       	0x40   
#define SM_PLUSV        	0x80   
#define SM_DACT         	0x100   
#define SM_SDIORD       	0x200   
#define SM_SDISHARE     	0x400   
#define SM_SDINEW       	0x800   
#define SM_ADPCM        	0x1000   
#define SM_ADPCM_HP     	0x2000 	

/* Private function prototypes -----------------------------------------------*/
void VS1003_Init(void);
void VS1003_SetVol(uint16_t volt);
void VS1003_RamTest(void);
void VS1003_SineTest(void);
void VS1003_SoftReset(void); 
void VS1003_Record_Init(void);
void VS1003_ResetDecodeTime(void);
uint16_t VS1003_GetDecodeTime(void);
void VS1003_WriteReg( uint8_t reg,uint16_t value );
uint16_t VS1003_ReadReg( uint8_t reg);
void VS1003_PlayBeep(void);
void VS1003_PlayBeep_DMA(void);
void VS1003_PlayMute_DMA(void);
void vs1003_gpio_init();

#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

