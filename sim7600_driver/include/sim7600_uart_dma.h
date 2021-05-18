/*
 * sim7600_uart_dma.h
 *
 *  Created on: May 2, 2021
 *      Author: Bom
 */

//dma need to define with circle gmode end trans single byte

//baudrate

#ifndef INC_SIM7600_UART_DMA_H_
#define INC_SIM7600_UART_DMA_H_
#include "main.h"
#include "stm32f407xx.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include "stdio.h"
#include "LL_GetTick.h"

#define Sim7600BaudDefaul 115200//115200
#define Sim7600BaudMain 3000000//3000000 //3 Mbps
//LL_USART_SetBaudRate (USART_TypeDef * USARTx, uint32_t PeriphClk,uint32_t OverSampling, uint32_t BaudRate)
#define pclk2Freq 84000000 //84MHz
//#define sim7600Baudrate(x) UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), x)
#define sim7600SetBaudrate(x) LL_USART_SetBaudRate(USART1, pclk2Freq, LL_USART_OVERSAMPLING_16, x)

#define pwrSIM_Pin LL_GPIO_PIN_0
#define pwrSIM_GPIO_Port GPIOE
#define rstSIM_Pin LL_GPIO_PIN_1
#define rstSIM_GPIO_Port GPIOE

#define semaphoreUART_TXwait_ms 100
#define semaphoreUART_updateRes_wait_ms 100
#define semaphoreCallHangUp_wait_ms 500
#define semaphorePlayMp3_wait_ms 10

//after max_num_restart_sim7600 + max_num_restart_sim7600 , but can't success
//sim7600 will sleep in ... minutes  (sleep_minutes_sim7600)
#define max_num_reset_sim7600 2
#define max_num_restart_sim7600 20
#define sleep_minutes_sim7600 12*60 //~12h //(max 1193 * 60)

enum {PLAY, STOP, RUNNING} statusPlay;

//max data length of response : \r\n ...respose...\r\n
#define max_data_length_of_response_r_n 100

//#define Sim_PWR(x) x ? HAL_GPIO_WritePin(pwrSIM_GPIO_Port, pwrSIM_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(pwrSIM_GPIO_Port, pwrSIM_Pin, GPIO_PIN_RESET)
#define Sim_PWR(x) x ? LL_GPIO_SetOutputPin(pwrSIM_GPIO_Port, pwrSIM_Pin) : LL_GPIO_ResetOutputPin(pwrSIM_GPIO_Port, pwrSIM_Pin)

//#define Sim_RST(x) x ? HAL_GPIO_WritePin(rstSIM_GPIO_Port, rstSIM_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(rstSIM_GPIO_Port, rstSIM_Pin, GPIO_PIN_RESET)
#define Sim_RST(x) x ? LL_GPIO_SetOutputPin(rstSIM_GPIO_Port, rstSIM_Pin) : LL_GPIO_ResetOutputPin(rstSIM_GPIO_Port, rstSIM_Pin)


//led status TCP connect, turn on when success, off when not
#define sim7600_tcp_led_status(x) x ? LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6) : LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6)
//led status toggle when tcp is connecting
#define sim7600_tcp_led_connecting_toggle LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_6)
#define sim7600_tcp_led_connecting_off LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7)
// Calculate length of statically allocated array

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

#define DMA_UART_SIM DMA2_Stream2

// Buffer for USART DMA
// Contains RAW unprocessed data received by UART and transfered by DMA

//don't want res1, res2 content be changed
const char* res1; int res1Length;
const char* res2; int res2Length;
uint32_t playTime; //in ms
int offsetTimer;

__STATIC_INLINE void sim7600_delay_ms(int _ms)
{
	osDelay(_ms);
}

// __STATIC_INLINE uint64_t sim7600GetTick_ms()
// {
//     return LL_GetTick_ms();
// }

//check string str contain substr ?
__STATIC_INLINE bool stringContain(const char* str, const char* substr)
{
 if(strstr(str, substr) != NULL) return true;
 return false;
}

//use this function after default dma and uasrt init
void sim7600_init();

void sim7600_usart_rx_check();
void sim7600_usart_send_byte(const void* data, int len);
void sim7600_usart_send_string(const char* str);
void sim7600_usart_IRQHandler(void);
void sim7600_change_baud(uint32_t baudrate);
void sim7600_restart();
bool sim7600_config();
int sim7600_handle_received_data();
bool sim7600_get_ipv4();
bool sim7600_open_network();
bool sim7600_open_tcp_connect();
bool sim7600_open_udp_connect();
void sim7600_handle_tcp_packet(uint8_t* tcpPacket, int length);
bool sim7600_send_cmd(const char *cmd, const char *response1, const char *response2, int timeout);
uint16_t sim7600_check_sum_data(uint8_t *data, int length);
bool sim7600_sync_timer_udp();
void sim7600_update_response(const char* _res1, const char* _res2);
void sim7600_handle_error();
void sim7600_keepAlive_udp();
bool sim7600_send_packet_ip(int type, uint8_t* data, int data_length);
void sim7600_handle_udp_packet(uint8_t* udpPacket, int length);
void playMp3DMA();

//min size of a packet mp3
#define min_mp3_udp_packet_size 24 + 10// header + 24B (1 frame 8kbps smallest bitrate)

#define packetMP3HeaderLength 10
#define checkSumHeaderLength 9

typedef struct
{
    uint16_t checkSumHeader; //encoded (reserved)
    int32_t IDframe; //ID of start frame
    int32_t songID;
	uint8_t frame[];
} __attribute__ ((packed)) packetMP3HeaderStruct;

#define mp3PacketFrameSize 432
typedef struct
{
    uint8_t frames[432]; //1 frame 144kbps
    int ID;
    bool IsEmpty;
} mp3PacketStruct;

#endif /* INC_SIM7600_UART_DMA_H_ */
