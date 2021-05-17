/*
 * sim7600_uart_dma.c
 *
 *  Created on: May 2, 2021
 *      Author: Bom
 */

#include "sim7600_uart_dma.h"

USART_TypeDef* usartSim7600 = USART1;
osSemaphoreId BinSemsim7600UartTxHandle;
osSemaphoreId BinSemPlayMp3Handle;
extern osMessageQId usart_rx_dma_queue_id;
extern enum {ON, OFF} eStatusPlayMp3;

const int sim_dma_buffer_size = 5000;
uint8_t sim_dma_buffer[5000]; //circle buffer
const int sim_dma_buffer_size_minus_1 = sim_dma_buffer_size - 1;

#define sim_buff_size  5000 //
#define sim_buff_size_minus_1 (sim_buff_size - 1)
uint8_t sim_buff[sim_buff_size + 1]; //real data received (+1 bytes for '\0' end of string)
//initialize sim_buff[sim_buff_size] = '\0' at sim7600_init
int sim_buff_length;

char sim7600_cmd_buff[200];

char serverDomain[100] = "iothtnhust20201.xyz";

int serverPort = 1308;

int restartSimstatus = 0; //0:none, 1:pwr done, 2: rst done

bool sim7600_error = false;
bool sim7600_network_IsOpen = false;
bool sim7600_tcp_IsOpen = false;
bool sim7600_udp_IsOpen = false;

bool HaveCall = false;

//init gpio, uart, dma(no fifo, byte->byte)

void sim7600_powerON()
{
  Sim_PWR(1);
  sim7600_delay_ms(500);

  Sim_PWR(0);
  sim7600_delay_ms(500);

  Sim_PWR(1);
  sim7600_delay_ms(16000);
}

void sim7600_powerOFF()
{
  Sim_PWR(1);
  sim7600_delay_ms(500);

  Sim_PWR(0);
  sim7600_delay_ms(4000);

  Sim_PWR(1);
  sim7600_delay_ms(26000);
}

void sim7600_reset()
{
  Sim_RST(1);
  sim7600_delay_ms(500);

  Sim_RST(0);
  sim7600_delay_ms(350);

  Sim_RST(1);
  sim7600_delay_ms(2000);
}

void sim7600_gpio_init()
{
  //gpio init
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);

  //set default state at begin (high or low depend on hardware / circuit)
  LL_GPIO_SetOutputPin(GPIOE, pwrSIM_Pin | rstSIM_Pin);

  GPIO_InitStruct.Pin = pwrSIM_Pin | rstSIM_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void sim7600_init()
{
  sim_buff[sim_buff_size] = '\0'; //initialize last bytes to ensure end of string (with strstr at handle receive data)

  //init semaphore to be ensure tx uart (share resource) use properly (place at main.c)
  osSemaphoreDef(BinSemsim7600UartTx);
  BinSemsim7600UartTxHandle = osSemaphoreCreate(osSemaphore(BinSemsim7600UartTx), 1);

  //semaphore to make sure that response is check properly
  osSemaphoreDef(BinSemPlayMp3);
  BinSemPlayMp3Handle = osSemaphoreCreate(osSemaphore(BinSemPlayMp3), 1);

  sim7600_gpio_init();

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  //uart1 + DMA2 stream 2, channel 2 init
  LL_USART_InitTypeDef USART_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  /*
       * USART1 GPIO Configuration
       *
       * PA9   ------> USART1_TX
       * PA10   ------> USART1_RX
       */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7; // AF_7 ~ USART1..3 see at datasheet (Figure.. Selecting an alternate function...)
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 RX DMA 2 stream 2 channel 4 Init */

  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_4);
  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_2);
  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)&USART1->DR);

  //set uart rx buffer receive
  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)sim_dma_buffer);
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, sim_dma_buffer_size);

  /* Enable HT & TC interrupts */
  //LL_DMA_EnableIT_HT(DMA2, LL_DMA_STREAM_2);
  //LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_2);

  /* DMA interrupt init */
  //NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));
  //NVIC_EnableIRQ(DMA2_Stream2_IRQn);

  /* USART configuration */
  USART_InitStruct.BaudRate = Sim7600BaudDefaul;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_EnableDMAReq_RX(USART1);
  LL_USART_EnableIT_IDLE(USART1);

  /* USART interrupt */
  //priority high (6) after spi and dma for vs1003
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* Enable USART and DMA */
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
  LL_USART_Enable(USART1);

  //power on sim7600
  sim7600_powerON();
  //then config
  //sim7600_config();
  // while(1)
  // {
	//  while(!LL_USART_IsActiveFlag_RXNE(usartSim7600));
  //    LL_USART_ReceiveData8(usartSim7600);
  // }
  //sim7600_usart_send_string("ATE0\r\n");
}

bool Sim7600BasicConfigSuccess = false;

bool sim7600_config()
{
  Sim7600BasicConfigSuccess = false;
  //config until success (connect to server)

  //echo cmd off
  if (!sim7600_send_cmd("ATE0\r\n", "OK", "", 500))
    return false;
  restartSimstatus = 0; //reset

  //change to main baudrate
  sprintf(sim7600_cmd_buff, "AT+IPR=%d\r\n", Sim7600BaudMain);
  if (!sim7600_send_cmd(sim7600_cmd_buff, "OK", "", 500))
    return false;
  sim7600_change_baud(Sim7600BaudMain);

  //check sim
  if (!sim7600_send_cmd("at+ciccid\r\n", "+ICCID:", "", 500))
    return false;
  if (!sim7600_send_cmd("at+csq\r\n", "+CSQ: ", "", 500))
    return false;

  // set timeout value for AT+NETOPEN/AT+CIPOPEN/AT+CIPSEND
  //AT+CIPTIMEOUT=10000,10000,5000 ~ 10s, 10s, 5s
  if (!sim7600_send_cmd("AT+CIPTIMEOUT=10000,10000,5000\r\n", "OK", "", 500))
    return false;

  //config parameters of socket
  //10 times retranmission IP packet, no(0) delay to output data received
  //ack=0, 1:error result code with string values
  //1:add data header, the format is â€œ+RECEIVE,<link num>,<data length>â€�
  //< AsyncMode > = 0
  //minimum retransmission timeout value for TCP connection in ms : 12000(max)
  if (!sim7600_send_cmd("AT+CIPCCFG=10,0,0,1,1,0,3000\r\n", "OK", "", 500))
    return false;

  //display header when receive â€œ+RECEIVE,<link num>,<data length>â€�
  //AT+CIPHEAD=1 : \r\nOK\r\n
  if (!sim7600_send_cmd("AT+CIPHEAD=1\r\n", "OK", "", 500))
    return false;

  //don't display remote IP (server ip)
  //AT+CIPSRIP=0 : \r\nOK\r\n
  if (!sim7600_send_cmd("AT+CIPSRIP=0\r\n", "OK", "", 500))
    return false;

  Sim7600BasicConfigSuccess = true;

  //open net
  if (!sim7600_open_network())
    return false;

  //open udp connect
  //create UDP connect socket at link 1, local port (mot important) 8080
  //AT+CIPOPEN=1,"UDP",,,8080 : \r\n+CIPOPEN: 1,0 or \r\n+CIPOPEN: 1,4
  if (!sim7600_open_udp_connect())
    return false;

  //open TCP socket with domain
  // if (!sim7600_open_tcp_connect())
  //   return false;

  //success config
  return true;
}

int UDPsendStatus = 0; // 0 : none , 1 : send smd, 2 : receive >, 3: send data, 4 : success
bool sim7600_sync_timer_udp()
{
  // //AT+CIPSEND=1,5,"45.118.145.137",1308
  // sprintf(sim7600_cmd_buff, "AT+CIPSEND=1,8,\"%s\",%d\r", serverDomain, serverPort);
  // int try = 5;
  // while(try--)
  // {
  //   sim7600_usart_send_string(sim7600_cmd_buff);
  //   UDPsendStatus = 1;
  //   sim7600_delay_ms(10);
  //   if(UDPsendStatus == 2)
  //   {
  //     timSim7600->CNT = 0; // reset timer
  //     UDPsendStatus = 3;
  //     sprintf(sim7600_cmd_buff, "gettime%d", try);
  //     sim7600_usart_send_string(sim7600_cmd_buff);
  //     sim7600_delay_ms(10);
  //     if (UDPsendStatus >= 4) //wait 1000 ms
  //     {
  //       sim7600_delay_ms(1000);
  //       if(UDPsendStatus == 5 + try) return true;
  //     }
  //   }
  // }
  return false;
}

//param bool openWithIPv4
//return: 0-success, 1:TCP error(net is good but can't connect). 2:error need to start
//

void sim7600_update_response(const char* _res1, const char* _res2)
{
  res1Length = strlen(_res1);
  res2Length = strlen(_res2);
  if(res1Length > 0)  res1 = _res1;
  else res1 = NULL;
  if(res2Length > 0) res2 = _res2;
  else res2 = NULL;
}

int sim7600_open_netStatus = 0; // 0 : none, 1 : send cmd, 2 : success
bool sim7600_open_network()
{
  //wait to aquire to update res
  osSemaphoreWait(BinSemsim7600UartTxHandle, semaphoreUART_TXwait_ms);

  sim7600_update_response("OK", "Network is already opened");

  sim7600_usart_send_string("AT+NETOPEN\r\n");

  sim7600_open_netStatus = 1;
  int try = 24; // ~ 12s
  do
  {
    sim7600_delay_ms(100); 
    if (sim7600_open_netStatus == 2)
    {
      //sim7600_open_netStatus = 0;
      sim7600_network_IsOpen = true;
      //return true;
      break;
    }
  } while (--try);

  sim7600_open_netStatus = 0;

  // if(!sim7600_network_IsOpen)//can't open network
  //   sim7600_error = true;
  sim7600_error = !sim7600_network_IsOpen;
  
  //release semaphore
  osSemaphoreRelease(BinSemsim7600UartTxHandle);

  return sim7600_network_IsOpen;
}

int sim7600_open_udp_connectStatus = 0; // 0 : none, 1 : send cmd, 2 : success
bool sim7600_open_udp_connect()
{
  //wait to aquire to update res
  osSemaphoreWait(BinSemsim7600UartTxHandle, semaphoreUART_TXwait_ms);

  sim7600_update_response("+CIPOPEN: 1,0", "+CIPOPEN: 1,4");

  sim7600_usart_send_string("AT+CIPOPEN=1,\"UDP\",,,8080\r\n");

  sim7600_open_udp_connectStatus = 1;
  int try = 24; // ~ 12s
  do
  {
    sim7600_delay_ms(100); 
    if (sim7600_open_udp_connectStatus == 2)
    {
      //sim7600_open_udp_connectStatus = 0;
      sim7600_udp_IsOpen = true;
      //return true;
    }
  } while (--try);

  sim7600_open_udp_connectStatus = 0;

  // if(!sim7600_udp_IsOpen)//can't open udp connect
  //   sim7600_error = true;
  sim7600_error = !sim7600_udp_IsOpen;

  //release semaphore
  osSemaphoreRelease(BinSemsim7600UartTxHandle);

  return sim7600_udp_IsOpen;
}

int sim7600_send_packetStatus = 0; // 0:none, 1:sendCMD, 2:received ">", 3:send data , 4:success

/* send udp data ("hello")
AT+CIPSEND=1,5,"45.118.145.137",1308<CR> //5:size of data in bytes
>hello : \r\nOK */

/* send tcp data ("hello")
AT+CIPSEND=0,5<CR> //5:size of data in bytes
>hello : \r\nOK */

//send packet, type:1 - UDP , type:0 - TCP
bool sim7600_send_packet_ip(int type, uint8_t* data, int data_length)
{
  //wait to aquire to update res
  osSemaphoreWait(BinSemsim7600UartTxHandle, osWaitForever);

  sim7600_update_response(">", "");
  if(type == 1) // udp
  {
    sprintf(sim7600_cmd_buff, "AT+CIPSEND=1,%d,\"%s\",%d\r", data_length, serverDomain, serverPort);
  }
  else if(type == 0) //tcp
  {
    sprintf(sim7600_cmd_buff, "AT+CIPSEND=0,%d\r", data_length);
  }
  else //ssl AT+CCHSEND=0,2
  {
	  sprintf(sim7600_cmd_buff, "AT+CCHSEND=0,%d\r", data_length);
  }

  sim7600_usart_send_string(sim7600_cmd_buff);
  sim7600_send_packetStatus = 1;

  bool send_ip_packet_success = false;
  int try = 60; // ~ 6s, since timeout set up for AT+CIPSEND is 5s
  do
  {
    sim7600_delay_ms(100); 
    if (sim7600_send_packetStatus == 2)
    {
      //send data
      sim7600_update_response("OK", "");
      sim7600_usart_send_byte(data, data_length);
      sim7600_send_packetStatus = 3;
    }
    else if(sim7600_send_packetStatus == 4) //success
    {
      //sim7600_send_packetStatus = 0;
      //release semaphore
      //osSemaphoreRelease(BinSemsim7600UartTxHandle);
      //return true;
      send_ip_packet_success = true;
      break;
    }
  } while (--try);

  sim7600_send_packetStatus = 0;

  //if(!send_ip_packet_success)//can't send
  // sim7600_error = true;

  //release semaphore
  osSemaphoreRelease(BinSemsim7600UartTxHandle);

  return send_ip_packet_success;
}

void sim7600_keepAlive_udp()
{
  static int keep_alive_udp_error = 0;
  //static firstSendNum = 10;
  
  //send udp, tcp keep alive every 30s
  if(sim7600_udp_IsOpen)
  {
    if(!sim7600_send_packet_ip(1, "00000002", 8)) keep_alive_udp_error++;
  }
  //after 2 time
  if(keep_alive_udp_error >= 2) 
  {
    keep_alive_udp_error = 0; // reset
    sim7600_udp_IsOpen = false;
    sim7600_error= true;
  }
  // if(firstSendNum > 0)
  // {
  //   firstSendNum--;
  //   osDelay(1000);
  // }
  // else
  //   osDelay(keep_alive_timeUDP_ms);
  
  // if(statusPlay == STOP)
  // {
  //   //reset
  //   firstSendNum = 10;
  //   osThreadSuspend(NULL); //self suspend
  // }
}

int sim7600_open_tcp_connectStatus = 0; // 1 : send cmd, 2 : success
bool sim7600_open_tcp_connect()
{
  //wait to aquire to update res
  osSemaphoreWait(BinSemsim7600UartTxHandle, semaphoreUART_TXwait_ms);

  sim7600_update_response("+CIPOPEN: 0,0", "+CIPOPEN: 0,4");
  sprintf(sim7600_cmd_buff, "AT+CIPOPEN=0,\"TCP\",\"%s\",%d\r\n", serverDomain, serverPort);

  sim7600_usart_send_string(sim7600_cmd_buff);

  sim7600_open_tcp_connectStatus = 1;
  int try = 24; // ~ 12s
  do
  {
    sim7600_delay_ms(500); //10ms receive max 100 bytes with baud = 115200
    sim7600_tcp_led_connecting_toggle;
    if (sim7600_open_tcp_connectStatus == 2)
    {
      //sim7600_open_tcp_connectStatus = 0;
      //turn on led
      sim7600_tcp_led_status(1);
      sim7600_tcp_IsOpen = true;
      //return true;
      break;
    }
  } while (--try);

  sim7600_open_tcp_connectStatus = 0;

  // if(!sim7600_tcp_IsOpen)//can't open tcp connect
  //   sim7600_error = true; 
  sim7600_error = !sim7600_tcp_IsOpen;

  //release semaphore
  osSemaphoreRelease(BinSemsim7600UartTxHandle);

  return sim7600_tcp_IsOpen;
}

bool sim7600_get_ipv4()
{
  //sprintf(sim7600_cmd_buff, "AT+CDNSGIP=\"")
      //sim7600_usart_send_string(serverDomain);
	return true;
}

//this function run in seperate task have highest priority
//error occur when network or tcp connect is close
void sim7600_handle_error()
{
  if(HaveCall)
  {
    HaveCall = false;

    //wait to aquire to send, wait until 
    osSemaphoreWait(BinSemsim7600UartTxHandle, semaphoreCallHangUp_wait_ms);
    sim7600_usart_send_string("AT+CHUP\r\n");
    //release semaphore
    osSemaphoreRelease(BinSemsim7600UartTxHandle);
  }
  if (sim7600_error)
  {
    //turn of led
    sim7600_tcp_led_status(0);
    //check whether basic config is success?
    //if success, check network again, if network false, restart, if still work, open tcp and udp again (then return -> don not restart)
    if (Sim7600BasicConfigSuccess)
    {
      //check network is open or closed(open again)
      if(!sim7600_network_IsOpen) sim7600_open_network();

      //after above step if network open success, continue open tcp
      if (sim7600_network_IsOpen)
      {
        //network still open, check udp, tcp connect
        if(!sim7600_udp_IsOpen) sim7600_open_udp_connect();
        //if(!sim7600_tcp_IsOpen) sim7600_open_tcp_connect();
        if(sim7600_udp_IsOpen) sim7600_error = false;
        return;
      }
    }

    //if basic config is not success or network false -> restart and config again
    //turn status (play->stop) and disable uart before restart
    sim7600_restart();
    if (sim7600_config())
      sim7600_error = false;
  }

  //update led error
  //if sim7600_error == true, turn on and vice versa
}

//send command (auto add "\r\n"), with timeout, if > timeout -> reset by
int cmdSendStatus = 0; // 0: none, 1: sended, 2: ok
bool sim7600_send_cmd(const char *cmd, const char *response1, const char *response2, int timeout)
{
  //wait to aquire to update res
  osSemaphoreWait(BinSemsim7600UartTxHandle, semaphoreUART_TXwait_ms);

  sim7600_update_response(response1, response2);
  int try = timeout / 50;
  bool send_cmd_success = false;
  do
  {
    sim7600_usart_send_string(cmd);

    cmdSendStatus = 1;
    sim7600_delay_ms(50); //10ms receive max 100 bytes with baud = 115200
    if (cmdSendStatus == 2)
    {
    	//cmdSendStatus = 0; // reset
      //return true;
      send_cmd_success = true;
      break;
    }
  } while (--try);

  cmdSendStatus = 0; // reset

  //release semaphore
  osSemaphoreRelease(BinSemsim7600UartTxHandle);

  return send_cmd_success;
}
//restart/reset module
//1st use pwr pin
//2nd use rst pin

void sim7600_restart()
{
  LL_USART_Disable(USART1);
  printf("%s", "rst\n");
  // reset all var status
  UDPsendStatus = 0;
  cmdSendStatus = 0;
  sim7600_open_tcp_connectStatus = 0;
  sim7600_open_udp_connectStatus = 0;
  sim7600_open_netStatus = 0;
  Sim7600BasicConfigSuccess = false;
  sim7600_error = false;
  sim7600_network_IsOpen = false;
  sim7600_tcp_IsOpen = false;
  sim7600_udp_IsOpen = false;
  sim7600_send_packetStatus = 0;


  if (restartSimstatus < max_num_restart_sim7600) //0-19
  {
    //use pwr pin to power off
    sim7600_powerOFF();
    restartSimstatus++;
  }
  else if (restartSimstatus < (max_num_restart_sim7600 + max_num_reset_sim7600)) // 20-21
  {
    sim7600_reset();
    sim7600_powerOFF();
    restartSimstatus++;
  }
  else // >=22
  {
    //delay to wait
    sim7600_delay_ms(sleep_minutes_sim7600 * 60 * 1000); // sleep_minutes_sim7600 minutes
    restartSimstatus = 0;
  }

  //power on again
  sim7600_powerON();
  //change baud rate to default
  sim7600_change_baud(Sim7600BaudDefaul); // in this function usart is enable again

  LL_USART_Enable(USART1);
}

void sim7600_usart_send_string(const char *str)
{
  sim7600_usart_send_byte(str, strlen(str));
}

// __STATIC_INLINE void LL_USART_TransmitData8(USART_TypeDef *USARTx, uint8_t Value)
// {
//   USARTx->DR = Value;
// }
// __STATIC_INLINE uint32_t LL_USART_IsActiveFlag_TXE(USART_TypeDef *USARTx)
// {
//   return (READ_BIT(USARTx->SR, USART_SR_TXE) == (USART_SR_TXE));
// }
// __STATIC_INLINE uint32_t LL_USART_IsActiveFlag_TC(USART_TypeDef *USARTx)
// {
//   return (READ_BIT(USARTx->SR, USART_SR_TC) == (USART_SR_TC));
// }
void sim7600_usart_send_byte(const void *data, int len)
{
  const uint8_t *d = data;

  for (; len > 0; --len, ++d)
  {
    LL_USART_TransmitData8(usartSim7600, *d);
    while (!LL_USART_IsActiveFlag_TXE(usartSim7600));
  }
  while (!LL_USART_IsActiveFlag_TC(usartSim7600));
}

// __STATIC_INLINE uint32_t LL_DMA_IsActiveFlag_TC1(DMA_TypeDef *DMAx)
// {
//   return (READ_BIT(DMAx->LISR, DMA_LISR_TCIF1) == (DMA_LISR_TCIF1));
// }
// __STATIC_INLINE void LL_DMA_ClearFlag_TC1(DMA_TypeDef *DMAx)
// {
//   WRITE_REG(DMAx->LIFCR, DMA_LIFCR_CTCIF1);
// }
//0 1 2 3 4
static volatile int old_pos = 0;
volatile int pos = 0;
void sim7600_usart_rx_check()
{
  /* Calculate current position in buffer */
  pos = sim_dma_buffer_size - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_2);
  //pos = sim_dma_buffer_size - dma_NDTR;
  if (pos != old_pos)
  { /* Check change in received data */
    if (pos > old_pos)
    { /* Current position is over previous one */
      /* We are in "linear" mode */
      /* Process data directly by subtracting "pointers" */
      //usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
      sim_buff_length = pos - old_pos;
      memcpy(sim_buff, sim_dma_buffer + old_pos, sim_buff_length);
    }
    else
    {
      /* We are in "overflow" mode */
      /* First process data to the end of buffer */
      sim_buff_length = sim_dma_buffer_size - old_pos;
      memcpy(sim_buff, sim_dma_buffer + old_pos, sim_buff_length);
      /* Check and continue with beginning of buffer */
      if (pos > 0)
      {
        //usart_process_data(&usart_rx_dma_buffer[0], pos);
        memcpy(sim_buff + sim_buff_length, sim_dma_buffer, pos);
        sim_buff_length += pos;
      }
    }
    //old_pos = pos; /* Save current position as old */
    old_pos += sim7600_handle_received_data();
    if(old_pos > sim_dma_buffer_size_minus_1) old_pos -= sim_dma_buffer_size; // ~ if(old_pos >= sim_dma_buffer_size) old_pos -= sim_dma_buffer_size;
  }
}

int numOfPacketUDPReceived = 0;
int lengthOfPacketUDPReceived = 0;
uint8_t printf_buff[20];
int errorSizeFrameNum = 0, errorSizeFrameNum2 = 0;
int errorSizeFrame;

//return 0 ~ success + continue, 1~ no success,2 ~ return sim_buff_index, 3~not sastify response
__STATIC_INLINE int check_normal_response(const char* response, int* sim_buff_index ) //macro
{
  uint8_t* posOfSubStr = strstr(sim_buff + (*sim_buff_index), response);
  if(posOfSubStr != NULL)
  {
    //check whether have \r\n at the end of this response
    posOfSubStr += strlen(response); //point to position right after the last character of response on sim_buff
    //in the worst case the last character of response is last character of sim_buff
    // ->  posOfSubStr point to sim_buff[sim_buff_length] = '\0' (initialize above) -> pointerTo_r_n == NULL
    uint8_t* pointerTo_r_n = strstr(posOfSubStr, "\r\n");
    if(pointerTo_r_n != NULL)
    {
      //perfect sastified 
      //change sim_buff_index
      *sim_buff_index = pointerTo_r_n - sim_buff + 2; // + 2 for "\r\n"
      return 0;
    }
    else if(posOfSubStr + max_data_length_of_response_r_n < sim_buff + sim_buff_length) //the worst case, have enough bytes but can't sastified
    {
      //data may be error-bit 
      *sim_buff_index = posOfSubStr - sim_buff + max_data_length_of_response_r_n;
      return 1;
    }
    else
      //return sim_buff_index; don't have enough data
      return 2;
  }
  return 3; 
}

//return num of bytes handled
int sim7600_handle_received_data()
{
  //handle received data

  //make sure sim_buff is string
  sim_buff[sim_buff_length] = '\0'; //can do this since real size of sim_buff = sim_buff_size + 1, so even sim_buff_length (max) = sim_buff_size, it is still oke
  uint8_t *posOfSubStr;
  int sim_buff_index = 0;

  memcpy(printf_buff, sim_buff, 19);
  printf("%s : %d", sim_buff, sim_buff_length);

  while (true)
  {
    posOfSubStr = NULL;
    if(sim_buff_index >= sim_buff_length) return sim_buff_length;

    //check cmd response
    if (cmdSendStatus || sim7600_open_tcp_connectStatus || sim7600_open_udp_connectStatus || sim7600_open_netStatus 
      || sim7600_send_packetStatus)
    {
      int resLengthTmp;
      if(res1 != NULL)
      {
        posOfSubStr = strstr(sim_buff + sim_buff_index, res1);
        resLengthTmp = res1Length;
      }
      if (posOfSubStr == NULL && res2 != NULL)
      {
          posOfSubStr = strstr(sim_buff + sim_buff_index, res2);
          resLengthTmp = res2Length;
      }
      if (posOfSubStr != NULL)
      {
        //check whether have \r\n at the end of this response
        posOfSubStr += resLengthTmp - 1; //point to the last charater of respose
        uint8_t* pointerTo_r_n = strstr(posOfSubStr, "\r\n");
        if(pointerTo_r_n != NULL)
        {
          if (cmdSendStatus == 1)
            cmdSendStatus = 2;
          else if (sim7600_open_tcp_connectStatus == 1)
            sim7600_open_tcp_connectStatus = 2;
          else if (sim7600_open_udp_connectStatus == 1)
            sim7600_open_udp_connectStatus = 2;
          else if (sim7600_open_netStatus == 1)
            sim7600_open_netStatus = 2;
          else if (sim7600_send_packetStatus == 1)
            sim7600_send_packetStatus = 2;
          else if (sim7600_send_packetStatus == 3)
            sim7600_send_packetStatus = 4;
          res1 = NULL;
          res2 = NULL;
          //change sim_buff_index
          sim_buff_index = pointerTo_r_n - sim_buff + 2; // + 2 for "\r\n"
          continue;
        }
        else if(posOfSubStr + max_data_length_of_response_r_n < sim_buff + sim_buff_length) //the worst case
        {
          //data may be error-bit 
          sim_buff_index = posOfSubStr - sim_buff + max_data_length_of_response_r_n;
          continue;
        }
        else //don't have enough data
          return sim_buff_index;
      }
    }

    //sms (reserved)

    //call (reserved)

    /// new receive UDP///////
    const char *receiveUdpRes = "+RECEIVE,1,";
    posOfSubStr = strstr(sim_buff + sim_buff_index, receiveUdpRes);
    if (posOfSubStr != NULL) //receive something from UDP port
    {
      //check whether have \r\n at buffer
      posOfSubStr += strlen(receiveUdpRes); //point to lenght of udp packet (right after ",1,")
      uint8_t* pointerTo_r_n = strstr(posOfSubStr, "\r\n"); //point to '\r'
      //in the worst case ',' after  "+RECEIVE,1" is last character of sim_buff
      // ->  posOfSubStr point to sim_buff[sim_buff_length] = '\0' (initialize above) -> pointerTo_r_n == NULL
      if (pointerTo_r_n != NULL)
      {
        int lengthOfUdpPacket = 0;
        while (posOfSubStr != pointerTo_r_n) //make sure break when meet '\r'
        {
          char numberTmp = (*posOfSubStr);
          if (numberTmp >= '0' && numberTmp <= '9')
          {
            lengthOfUdpPacket = lengthOfUdpPacket * 10 + numberTmp - '0';
          }
          else //data maybe bit-error
          {
            sim_buff_index = pointerTo_r_n - sim_buff + 2;// +2 since "\r\n"
            continue;
          }
          posOfSubStr++;
        }
        posOfSubStr += 2; //point to data
        if(posOfSubStr + lengthOfUdpPacket > sim_buff + sim_buff_length) // don't have enough data
          return sim_buff_index;
        //else data is sastified
        sim7600_handle_udp_packet(posOfSubStr, lengthOfUdpPacket);
        sim_buff_index = posOfSubStr - sim_buff + lengthOfUdpPacket;
        continue;
      }
      else if(posOfSubStr + 6 < sim_buff + sim_buff_length) //the worst case: +RECEIVE,1,9999\r\n
      {
        //data may be error-bit 
        sim_buff_index = posOfSubStr - sim_buff + 6;
        continue;
      }
      else 
        return sim_buff_index;  //dont have enough data
    }

    int resultTmp;

    //received error
    //when network error (many reason, one of those is "sim not found") : \r\n+CIPEVENT: NETWORK CLOSED UNEXPECTEDLY
    //then restart module with pwr or rst pin
    const char* netErrorRes = "+CIPEVENT: NETWORK CLOSED UNEXPECTEDLY";
    resultTmp = check_normal_response(netErrorRes, &sim_buff_index);
    if(resultTmp == 3){} //reponse can't find -> do nothing
    else if(resultTmp == 0) // success find
    {
      //sim7600 error, handle in a task sim7600 config task
      sim7600_network_IsOpen = false;
      sim7600_udp_IsOpen = false;
      sim7600_tcp_IsOpen = false;

      sim7600_error = true;
      continue;
    }
    //data maybe bit-error sim_buff_index = posOfSubStr - sim_buff + max_data_length_of_response_r_n;
    else if(resultTmp == 1) continue;
    //dont have enough data
    else if(resultTmp == 2) return sim_buff_index;

    // if(posOfSubStr != NULL)
    // {
    //   //check whether have \r\n at the end of this response
    //   posOfSubStr += strlen(netErrorRes);
    //   if(posOfSubStr < sim_buff + sim_buff_length)
    //   {
    //     uint8_t* pointerTo_r_n = strstr(posOfSubStr, "\r\n");
    //     if(pointerTo_r_n != NULL)
    //     {

    //       //sim7600 error, handle in a task sim7600 config task
    //       sim7600_network_IsOpen = false;
    //       sim7600_udp_IsOpen = false;
    //       sim7600_tcp_IsOpen = false;

    //       sim7600_error = true;

    //       //change sim_buff_index
    //       sim_buff_index = pointerTo_r_n - sim_buff + 2; // + 2 for "\r\n"
    //       continue; //success
    //     }
    //     else if(posOfSubStr + max_data_length_of_response_r_n < sim_buff + sim_buff_length) //the worst case
    //     {
    //       //data may be error-bit 
    //       sim_buff_index = posOfSubStr - sim_buff + max_data_length_of_response_r_n;
    //       continue;
    //     }
    //     else
    //       return sim_buff_index;
    //   }
    //   else return sim_buff_index; //don't have enough data
    // }


    // //when tcp connect is close +IPCLOSE: 0
    // const char* tcpErrorRes = "\r\n+IPCLOSE: 0";
    // //if(stringContain(sim_buff, tcpErrorRes))
    // if(strncmp(sim_buff, tcpErrorRes, strlen(tcpErrorRes)) == 0)
    // {
    //   memset(sim_buff, 0, sim_buff_length);
    //   //sim7600 error, handle in a task sim7600 config task
    //   sim7600_tcp_IsOpen = false;
    //   sim7600_error = true;
    //   //return;
    // }

    // //when udp connect is close +IPCLOSE: 1
    // const char* udpErrorRes = "\r\n+IPCLOSE: 1";
    // if(strncmp(sim_buff, udpErrorRes, strlen(udpErrorRes)) == 0)
    // //if(stringContain(sim_buff, udpErrorRes))
    // {
    //   memset(sim_buff, 0, sim_buff_length);
    //   //sim7600 error, handle in a task sim7600 config task
    //   sim7600_udp_IsOpen = false;
    //   sim7600_error = true;
    //   return;
    // }

    // //+CIPOPEN: 0,2
    // //tcp connect can't open since network is close
    // const char* netTcpCloseRes = "\r\n+CIPOPEN: 0,2";
    // if(strncmp(sim_buff, netTcpCloseRes, strlen(netTcpCloseRes)) == 0)
    // //if(stringContain(sim_buff, netTcpCloseRes))
    // {
    //   memset(sim_buff, 0, sim_buff_length);
    //   //sim7600 error, handle in a task sim7600 config task
    //   sim7600_network_IsOpen = false;
    //   sim7600_tcp_IsOpen = false;
    //   sim7600_error = true;
    //   return;
    // }

    // //+CIPOPEN: 1,2
    // //udp connect can't open since network is close
    // const char* netUdpCloseRes = "\r\n+CIPOPEN: 1,2";
    // if(strncmp(sim_buff, netUdpCloseRes, strlen(netUdpCloseRes)) == 0)
    // //if(stringContain(sim_buff, netUdpCloseRes))
    // {
    //   memset(sim_buff, 0, sim_buff_length);
    //   //sim7600 error, handle in a task sim7600 config task
    //   sim7600_network_IsOpen = false;
    //   sim7600_udp_IsOpen = false;
    //   sim7600_error = true;
    //   return;
    // }

    // //when receive call, send AT+CHUP to hang up
    // //RING
    // if(stringContain(sim_buff, "RING"))
    // {
    //   memset(sim_buff, 0, sim_buff_length);
    //   //sim7600 error, handle in a task sim7600 config task
    //   HaveCall = true;
    //   return;
    // }
    break;
  }
  
}

int error_frame = 0;
int miss_frame = 0;
int late_frame = 0;
int old_ID_frame = 0;
void sim7600_handle_udp_packet(uint8_t* udpPacket, int length)
{
  eStatusPlayMp3 = ON;
  //wait to aquire to update res
  osSemaphoreWait(BinSemPlayMp3Handle, semaphorePlayMp3_wait_ms);

  packetMP3HeaderStruct *packetMP3Header = (packetMP3HeaderStruct*)udpPacket;
  //check sum to confirm it is head of a packet
  if (packetMP3Header->checkSumHeader == sim7600_check_sum_data(udpPacket + 2, length - 2))
  {
    numOfPacketUDPReceived++;
    uint32_t IDtmp = packetMP3Header->IDframe;
    if(IDtmp > old_ID_frame)
    {
      miss_frame += IDtmp - old_ID_frame - 1;
      old_ID_frame = IDtmp;
    }
    else
    {
      late_frame++;
    }
    // if(packetMP3Header->songID != songID)
    // {
    //   songID = packetMP3Header->songID;
    //   //clear status of mp3Packet
    //   int i, limit = mp3PacketSize;
    //   for(i = 0; i < limit; i++) mp3Packet[i].IsEmpty = true;
    // }   

    // //copy frame
    // index_mp3_packet = packetMP3Header->IDframe % mp3PacketSize;
    // uint8_t tmp = *(packetMP3Header->frame);
    // //if (mp3Packet[index_mp3_packet].IsEmpty == true)
    // {
    //   mp3Packet[index_mp3_packet].IsEmpty = false;
    //   //clear first
    //   //memset(mp3Packet[index_mp3_packet].frames, 0, mp3PacketFrameSize);
    //   for(int i = 0; i < mp3PacketFrameSize; i++) mp3Packet[index_mp3_packet].frames[i] = 0;
    //   mp3Packet[index_mp3_packet].length = length - packetMP3HeaderLength;
    //   memcpy(mp3Packet[index_mp3_packet].frames, packetMP3Header->frame, length - packetMP3HeaderLength);
    //   if (!IsPlaying && index_mp3_packet > 10)
    //   {
    //     //IsPlaying = true;
    //     //playMp3DMA();
    //   }
    // }
  }
  else
  {
	  error_frame++;
  }

  //release semaphore
  osSemaphoreRelease(BinSemPlayMp3Handle);
}

uint16_t sim7600_check_sum_data(uint8_t *ptr, int length)
{
	uint32_t checksum = 0;

	while (length > 1) //cong het cac byte16 lai
	{
		checksum += ((uint32_t)*ptr << 8) | (uint32_t) *(ptr + 1);
		ptr += 2;
		length -= 2;
	}
	if (length)
	{
		checksum += ((uint32_t)*ptr) << 8; //neu con le 1 byte
	}
	while (checksum >> 16)
	{
		checksum = (checksum & 0xFFFF) + (checksum >> 16);
	}
	//nghich dao bit
	checksum = ~checksum;
	//hoan vi byte thap byte cao
	return (uint16_t)checksum;
}

bool IsHaveHeaderOfApacket = false;
int remainDataInApacket;
int currentSession = 0;
int currentIDframe = 0;

bool CheckSumCMD(uint8_t* tcpPacket, int length)
{
  uint16_t* checksump = (uint16_t*)(tcpPacket + length - 2);
  uint16_t checksum = *checksump;
  if(checksum == sim7600_check_sum_data(tcpPacket, length - 2)) return true;
  return false;
}

int remainBytesInApacket;
void sim7600_handle_tcp_packet(uint8_t* tcpPacket, int length)
{
  //check is command
  /*if(length < 50)
  {
    if(strncmp(tcpPacket, "cmd", 3) == 0)
    {
      //checkSum cmd
      if(CheckSumCMD(tcpPacket, length))
      {
        tcpPacket += 3; //ignore cmd
        if(strncmp(tcpPacket, "play", 4) == 0)
        {
          //play mp3
          uint32_t* playTimeP = tcpPacket + 4;
          playTime = *playTimeP;
          IDframeNeed = 0;
          mp3packetIndex = 0;
          if(statusPlay == STOP)
            statusPlay = PLAY;
          return;
        }
        else if(strncmp(tcpPacket, "stop", 4) == 0)
        {
          //stop
          statusPlay = STOP;
          return;
        }
        else if(strncmp(tcpPacket, "gettime", 7) == 0)
        {
          //get time with udp
          return;
        }
      }
    }
     
  }


  //handle mp3 packet
  if (length > packetMP3HeaderLength)
  {
    //find header of each packet
    if (tcpPacket[0] == 1) //maybe is head of mp3 packet
    {
      struct packetMP3HeaderStruct *packetMP3Header = (struct packetMP3HeaderStruct*)tcpPacket;
      //check sum to confirm it is head of a packet
      if(packetMP3Header->checkSumHeader == sim7600_check_sum_data(tcpPacket, checkSumHeaderLength))
      {
        if(packetMP3Header->IDframe < IDframeNeed) return;
        FrameOffset = packetMP3Header->IDframe;
        mp3Packet[mp3packetIndex].length = packetMP3Header->totalBytes; //not include header
        //mp3Packet[mp3packetIndex].numOfFrame = packetMP3Header->numOfFrame;

        //copy frame
        memcpy(mp3Packet[mp3packetIndex].frames, tcpPacket + packetMP3HeaderLength, length - packetMP3HeaderLength);
        remainBytesInApacket = mp3Packet[mp3packetIndex].length - (length - packetMP3HeaderLength);
        if(remainBytesInApacket == 0)
        {
          mp3Packet[mp3packetIndex].IsEmpty = false;
          mp3packetIndex++;
          mp3packetIndex %= 3;
          IsHaveHeaderOfApacket = false;
          return;
        }
        //else
        IsHaveHeaderOfApacket = true;
        return;
      }
    }
  }

  //handle remain part of a packet
  if (IsHaveHeaderOfApacket && length <= remainBytesInApacket) //remain part of a packet (confirmed header)
  {
    memcpy(mp3Packet[mp3packetIndex].frames + (mp3Packet[mp3packetIndex].length - remainBytesInApacket), tcpPacket, length);
    remainBytesInApacket -= length;
    if (remainBytesInApacket == 0)
    {
      mp3Packet[mp3packetIndex].IsEmpty = false;
      mp3packetIndex++;
      mp3packetIndex %= 3;
      IsHaveHeaderOfApacket = false;
      return;
    }
    return;
  }*/
}

/**
 * \brief           usartSim7600 global interrupt handler
 */
void sim7600_usart_IRQHandler(void)
{
  /* Check for IDLE line interrupt */
  //if (LL_USART_IsEnabledIT_IDLE(usartSim7600) && LL_USART_IsActiveFlag_IDLE(usartSim7600)) {
  //    LL_USART_ClearFlag_IDLE(usartSim7600);        /* Clear IDLE line flag */
  //   sim7600_usart_rx_check();                       /* Check for data to process */
  //}

  if ((usartSim7600->CR1 & USART_CR1_IDLEIE) == USART_CR1_IDLEIE && (usartSim7600->SR & USART_SR_IDLE) == (USART_SR_IDLE))
  {
    // Clear IDLE line flag
    volatile uint32_t tmpreg;
    tmpreg = usartSim7600->SR;
    (void)tmpreg;
    tmpreg = usartSim7600->DR;
    (void)tmpreg;

    //sim7600_usart_rx_check(); // Check for data to process
    //put data to queue to invoke task handle rx data
    osMessagePut(usart_rx_dma_queue_id, 1, 0);
  }
}

void sim7600_change_baud(uint32_t baudrate)
{
  LL_USART_Disable(USART1);
  sim7600SetBaudrate(baudrate);
  LL_USART_Enable(USART1);
}
