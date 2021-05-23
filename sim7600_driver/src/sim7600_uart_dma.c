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

extern osThreadId usart_rx_dmaHandle;

extern osMessageQId usart_rx_dma_queue_id;
extern osMessageQId play_mp3_queue_id;

extern enum {ON, OFF} eStatusPlayMp3;

#define sim_dma_buffer_size 5000
uint8_t sim_dma_buffer[sim_dma_buffer_size]; //circle buffer
#define sim_dma_buffer_size_minus_1  (sim_dma_buffer_size - 1)
uint8_t* sim_dma_buffer_pointer = sim_dma_buffer;

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

bool sim7600_have_call = false;

#define mp3PacketSize 24
mp3PacketStruct mp3Packet[mp3PacketSize];

//init gpio, uart, dma(no fifo, byte->byte)

void sim7600_powerON()
{
  Sim_PWR(1);
  sim7600_delay_ms(500);

  Sim_PWR(0);
  sim7600_delay_ms(500);

  Sim_PWR(1);
  sim7600_delay_ms(20000);
}

void sim7600_powerOFF()
{
  Sim_PWR(1);
  sim7600_delay_ms(500);

  Sim_PWR(0);
  sim7600_delay_ms(4000);

  Sim_PWR(1);
  sim7600_delay_ms(20000); // ~ 3 minutes
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
  LL_GPIO_SetOutputPin(GPIOE, pwrSIM_Pin | rstSIM_Pin | CTS_SIM_Pin);

  GPIO_InitStruct.Pin = pwrSIM_Pin | rstSIM_Pin | CTS_SIM_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void sim7600_init()
{
  //init udp var
  // int i, limit = mp3PacketSize;
  // for(i = 0; i < limit; i++) mp3Packet[i].IsEmpty = true;

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
  //NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
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
  LL_USART_EnableRTSHWFlowCtrl(USART1);

  /* USART interrupt */
  //priority high (6) after spi and dma for vs1003
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 14, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  // Clear IDLE line flag
  volatile uint32_t tmpreg;
  tmpreg = usartSim7600->SR;
  (void)tmpreg;
  tmpreg = usartSim7600->DR;
  (void)tmpreg;

  /* Enable USART and DMA */
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
  LL_USART_Enable(USART1);

  //wait until USART DMA is ready
  while(!LL_USART_IsEnabled(USART1) || !LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_2));

  //power off to debug (don't need to plug out sim7600)
  sim7600_powerOFF();

  //power on sim7600
  sim7600_powerON();
}

bool Sim7600BasicConfigSuccess = false;

bool sim7600_config()
{
  //osThreadResume(usart_rx_dmaHandle);

  Sim7600BasicConfigSuccess = false;
  //config until success (connect to server)

  //echo cmd off
  if (!sim7600_send_cmd("ATE0\r\n", "OK", "", 500))
    return false;
  restartSimstatus = 0; //reset

  //flow control AT+IFC=0,2 (CTS at sim , and RTS at stm32) at pin pe1
  if (!sim7600_send_cmd("AT+IFC=0,2\r\n", "OK", "", 500))
    return false;

  //change to main baudrate
  sprintf(sim7600_cmd_buff, "AT+IPR=%d\r\n", Sim7600BaudMain);
  if (!sim7600_send_cmd(sim7600_cmd_buff, "OK", "", 500))
    return false;
  sim7600_change_baud(Sim7600BaudMain);

  //check sim
  if (!sim7600_send_cmd("at+ciccid\r\n", "OK", "", 500))
    return false;
  if (!sim7600_send_cmd("at+csq\r\n", "OK", "", 500))
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

int sim7600_open_netStatus = 0; // 0 : success, 1 : send cmd
bool sim7600_open_network()
{
  //wait to aquire to update res
  osSemaphoreWait(BinSemsim7600UartTxHandle, osWaitForever);

  sim7600_update_response("OK", "Network is already opened");

  sim7600_usart_send_string("AT+NETOPEN\r\n");

  sim7600_open_netStatus = 1;

  //release semaphore
  osSemaphoreRelease(BinSemsim7600UartTxHandle);

  int try = 24; // ~ 12s
  do
  {
    sim7600_delay_ms(500);
    if (sim7600_open_netStatus == 0)
    {
      sim7600_network_IsOpen = true;
      break;
    }
  } while (--try);

  sim7600_open_netStatus = 0;

  sim7600_error = !sim7600_network_IsOpen;

  return sim7600_network_IsOpen;
}

int sim7600_open_udp_connectStatus = 0; // 1 : send cmd, 0 : success
bool sim7600_open_udp_connect()
{
  //wait to aquire to update res
  osSemaphoreWait(BinSemsim7600UartTxHandle, osWaitForever);

  sim7600_update_response("+CIPOPEN: 1,0", "+CIPOPEN: 1,4");

  sim7600_usart_send_string("AT+CIPOPEN=1,\"UDP\",,,8080\r\n");

  sim7600_open_udp_connectStatus = 1;

  //release semaphore
  osSemaphoreRelease(BinSemsim7600UartTxHandle);

  int try = 24; // ~ 12s
  do
  {
    sim7600_delay_ms(500);
    if (sim7600_open_udp_connectStatus == 0)
    {
      sim7600_udp_IsOpen = true;
      sim7600_tcp_led_status(1);
    }
  } while (--try);

  sim7600_open_udp_connectStatus = 0;

  sim7600_error = !sim7600_udp_IsOpen;

  return sim7600_udp_IsOpen;
}

int sim7600_send_packetStatus1 = 0, sim7600_send_packetStatus2 = 0; // 1:send, 0:succes  //status1: send cmd, status2: send data

/* send udp data ("hello")
AT+CIPSEND=1,5,"45.118.145.137",1308<CR> //5:size of data in bytes
>hello : \r\nOK */

/* send tcp data ("hello")
AT+CIPSEND=0,5<CR> //5:size of data in bytes
>hello : \r\nOK */

/*
//send packet, type:1 - UDP , type:0 - TCP
bool sim7600_send_packet_ip(int type, uint8_t* data, int data_length)
{
  //sim7600_update_response(">", "");
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

  //wait to aquire to update res
  osSemaphoreWait(BinSemsim7600UartTxHandle, osWaitForever);

  sim7600_usart_send_string(sim7600_cmd_buff);
  osDelay(1);
  sim7600_update_response("OK", "");
  sim7600_usart_send_byte(data, data_length);
  sim7600_send_packetStatus = 3;

  //release semaphore
  osSemaphoreRelease(BinSemsim7600UartTxHandle);

  bool send_ip_packet_success = false;
  int try = 30; // ~ 6s, since timeout set up for AT+CIPSEND is 5s
  do
  {
    sim7600_delay_ms(200);
    if(sim7600_send_packetStatus == 4) //success
    {
      send_ip_packet_success = true;
      break;
    }
  } while (--try);

  sim7600_send_packetStatus = 0;

  return send_ip_packet_success;
}*/

//send packet, type:1 - UDP , type:0 - TCP
bool sim7600_send_packet_ip(int type, uint8_t* data, int data_length)
{
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

  //wait to aquire to update res
  osSemaphoreWait(BinSemsim7600UartTxHandle, osWaitForever);

  sim7600_usart_send_string(sim7600_cmd_buff);
  sim7600_send_packetStatus1 = 1;

  //release semaphore
  osSemaphoreRelease(BinSemsim7600UartTxHandle);

  bool send_ip_packet_success = false;
  int try = 5;

  do
  {
    sim7600_delay_ms(100);
    if(sim7600_send_packetStatus1 == 0) //success
    {
      send_ip_packet_success = true;
      break;
    }
  } while (--try);

  sim7600_send_packetStatus1 = 0;


  if(send_ip_packet_success)
  {
    send_ip_packet_success = false; //reset

    try = 60; // ~ 6s, since timeout set up for AT+CIPSEND is 5s

    //send data
    //wait to aquire to update res
    osSemaphoreWait(BinSemsim7600UartTxHandle, osWaitForever);

    sim7600_update_response("OK", "");
    sim7600_usart_send_byte(data, data_length);
    sim7600_send_packetStatus2 = 1;

    //release semaphore
    osSemaphoreRelease(BinSemsim7600UartTxHandle);

    do
    {
      sim7600_delay_ms(100);
      if(sim7600_send_packetStatus2 == 0) //success
      {
        send_ip_packet_success = true;
        break;
      }
    } while (--try);

    sim7600_send_packetStatus2 = 0;
  }

  return send_ip_packet_success;
}

int keep_alive_udp_error = 0;
void sim7600_keepAlive_udp()
{
  //static firstSendNum = 10;

  //send udp, tcp keep alive every 30s
  if(sim7600_udp_IsOpen)
  {
    if(!sim7600_send_packet_ip(1, "00000002", 8)) keep_alive_udp_error++;
    else 
    {
      keep_alive_udp_error = 0;
      printf("request udp done\n");
    }
  }
  //after 2 time
  if(keep_alive_udp_error >= 10)
  {
    keep_alive_udp_error = 0; // reset
    sim7600_udp_IsOpen = false;
    sim7600_error= true;
  }
}

int sim7600_open_tcp_connectStatus = 0; // 1 : send cmd, 0 : success
bool sim7600_open_tcp_connect()
{
  sprintf(sim7600_cmd_buff, "AT+CIPOPEN=0,\"TCP\",\"%s\",%d\r\n", serverDomain, serverPort);

  //wait to aquire to update res
  osSemaphoreWait(BinSemsim7600UartTxHandle, osWaitForever);

  sim7600_update_response("+CIPOPEN: 0,0", "+CIPOPEN: 0,4");
  sim7600_usart_send_string(sim7600_cmd_buff);
  sim7600_open_tcp_connectStatus = 1;

  //release semaphore
  osSemaphoreRelease(BinSemsim7600UartTxHandle);

  int try = 120; // ~ 12s
  do
  {
    sim7600_delay_ms(100); //10ms receive max 100 bytes with baud = 115200
    sim7600_tcp_led_connecting_toggle;
    if (sim7600_open_tcp_connectStatus == 0)
    {
      //turn on led
      sim7600_tcp_led_status(1);
      sim7600_tcp_IsOpen = true;
      //return true;
      break;
    }
  } while (--try);

  sim7600_open_tcp_connectStatus = 0; //reset

  // if(!sim7600_tcp_IsOpen)//can't open tcp connect
  //   sim7600_error = true;
  sim7600_error = !sim7600_tcp_IsOpen;

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
  if(sim7600_have_call)
  {
    sim7600_have_call = false;

    //wait to aquire to send, wait until
    osSemaphoreWait(BinSemsim7600UartTxHandle, osWaitForever);
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
        if(sim7600_udp_IsOpen)
        {
        	sim7600_error = false;
        	return; //when network and udp is ok
        }
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
int cmdSendStatus = 0; // 1: sended, 0: ok
bool sim7600_send_cmd(const char *cmd, const char *response1, const char *response2, int timeout)
{
  

  sim7600_update_response(response1, response2);
  int try = timeout / 100;
  bool send_cmd_success = false;
  do
  {
    //wait to aquire to update res
    osSemaphoreWait(BinSemsim7600UartTxHandle, osWaitForever);

    sim7600_usart_send_string(cmd);

    //release semaphore
    osSemaphoreRelease(BinSemsim7600UartTxHandle);

    cmdSendStatus = 1;
    sim7600_delay_ms(100); //10ms receive max 100 bytes with baud = 115200
    if (cmdSendStatus == 0)
    {
      send_cmd_success = true;
      break;
    }
  } while (--try);

  cmdSendStatus = 0; // reset

  return send_cmd_success;
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
int old_pos = 0;
int pos = 0;

volatile uint16_t ndtr_dma, ndtr_dma2;
volatile int returnTmp = 0;
void sim7600_usart_rx_check()
{
  /* Calculate current position in buffer */
  ndtr_dma = LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_2);
  pos = sim_dma_buffer_size - (int)ndtr_dma;
  // if(pos > (2*sim_dma_buffer_size))
  // {
	//   pos = 0;
  // }
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
    returnTmp = sim7600_handle_received_data();
    if(returnTmp < 0) //something wrong
    {
    	old_pos = pos;
    	return;
    }
    // if(old_pos > (2*sim_dma_buffer_size))
    // {
    // 	old_pos = 0;
    // }
    old_pos += returnTmp;
    old_pos %= sim_dma_buffer_size; // ~ if(old_pos >= sim_dma_buffer_size) old_pos -= sim_dma_buffer_size;
    //sim_buff[sim_buff_length] = '\0';
    //printf("%s", sim_buff);
  }
  //ndtr_dma2 = LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_2);
  /*if(ndtr_dma2 != ndtr_dma)
  {
	  printf("error");
  }*/
  //sim7600_resume_rx_uart_dma(ndtr_dma, old_pos);
}

int numOfPacketUDPReceived = 0;
int lengthOfPacketUDPReceived = 0;
char printf_buff[20];
int errorSizeFrameNum = 0, errorSizeFrameNum2 = 0;
int errorSizeFrame;

//return 0 ~ success + continue, 1~ no success,2 ~ return sim_buff_index
__STATIC_INLINE int check_normal_response(uint8_t* posOfSubStr, const char* response, int* sim_buff_index ) //macro
{
  //check whether have \r\n at the end of this response
  posOfSubStr += strlen(response); //point to position right after the last character of response on sim_buff
  //in the worst case the last character of response is last character of sim_buff
  // ->  posOfSubStr point to sim_buff[sim_buff_length] = '\0' (initialize above) -> pointerTo_r_n == NULL
  uint8_t *pointerTo_r_n = strstr(posOfSubStr, "\r\n");
  if (pointerTo_r_n != NULL)
  {
    //perfect sastified
    //change sim_buff_index
    *sim_buff_index = pointerTo_r_n - sim_buff + 2; // + 2 for "\r\n"
    return 0;
  }
  else if (posOfSubStr + max_data_length_of_response_r_n < sim_buff + sim_buff_length) //the worst case, have enough bytes but can't sastified
  {
    //data may be error-bit
    *sim_buff_index = posOfSubStr - sim_buff + max_data_length_of_response_r_n;
    return 1;
  }
  else
    //return sim_buff_index; don't have enough data
    return 2;
}

//list response
#define listResponseSize 7
char listResponse[listResponseSize][20] =
{                 //result check
  "+RECEIVE,1,",  //0 udp received
  "+CIPEVENT",    //1 net error
  "+IPCLOSE: 1",  //2 udp closed
  "+CIPOPEN: 1,2", //3 can't open udp since network
  "RING",         //4 call
  "+CMTI",        //5 sms
  "\r\n>"         //6 signal send data udp / tcp
                  //.. cmd check
};

//return num of bytes handled
int sim7600_handle_received_data()
{
  //handle received data

  //make sure sim_buff is string
  sim_buff[sim_buff_length] = '\0'; //can do this since real size of sim_buff = sim_buff_size + 1, so even sim_buff_length (max) = sim_buff_size, it is still oke
  uint8_t* posOfSubStr = NULL;
  uint8_t* posOfSubStrSave = NULL;
  int sim_buff_index = 0;

  while (true)
  {
    posOfSubStr = NULL;
    posOfSubStrSave = NULL;
    if(sim_buff_index >= sim_buff_length) return sim_buff_length;
    if(*(sim_buff + sim_buff_index) == '\0') //ignore wrong data
    {
    	sim_buff_index++;
    	continue;
    }

    //memcpy(printf_buff, sim_buff + sim_buff_index, 9);
    //printf("%s : %d", printf_buff, sim_buff_length);
    //check what response is appeared earliest

    int resultCheck = -1;
    //first cmd response
    int checkCmdResLength;
    if (cmdSendStatus || sim7600_open_tcp_connectStatus || sim7600_open_udp_connectStatus || sim7600_open_netStatus
      || sim7600_send_packetStatus2)
    {
      if(res1 != NULL)
      {
        posOfSubStr = strstr(sim_buff + sim_buff_index, res1);
        checkCmdResLength = res1Length;
      }
      if (posOfSubStr == NULL && res2 != NULL)
      {
          posOfSubStr = strstr(sim_buff + sim_buff_index, res2);
          checkCmdResLength = res2Length;
      }
    }
    if(posOfSubStr != NULL)
    {
      posOfSubStrSave = posOfSubStr;
      resultCheck = listResponseSize;
    }

    //check in list
    for(int i = 0; i < listResponseSize; i++)
    {
      posOfSubStr = strstr(sim_buff + sim_buff_index, listResponse[i]);
      if(posOfSubStr != NULL)
      {
        if(posOfSubStrSave == NULL)
        {
          posOfSubStrSave = posOfSubStr;
          resultCheck = i;
        }
        //else posOfSubStrSave = (posOfSubStr < posOfSubStrSave) ? posOfSubStr : posOfSubStrSave;
        else
        {
          if(posOfSubStr < posOfSubStrSave)
          {
            posOfSubStrSave = posOfSubStr;
            resultCheck = i;
          }
        }
      }
    }

    int resultTmp = -1;
    if (resultCheck < 0){break;} // nothing
    else if (resultCheck == 0) //udp received
    {
      //check whether have \r\n at buffer
      posOfSubStrSave += strlen(listResponse[resultCheck]);                 //point to lenght of udp packet (right after ",1,")
      uint8_t *pointerTo_r_n = strstr(posOfSubStrSave, "\r\n"); //point to '\r'
      //in the worst case ',' after  "+RECEIVE,1" is last character of sim_buff
      // ->  posOfSubStrSave point to sim_buff[sim_buff_length] = '\0' (initialize above) -> pointerTo_r_n == NULL
      if (pointerTo_r_n != NULL)
      {
        int lengthOfUdpPacket = 0;
        while (posOfSubStrSave != pointerTo_r_n) //make sure break when meet '\r'
        {
          char numberTmp = (*posOfSubStrSave);
          if (numberTmp >= '0' && numberTmp <= '9')
          {
            lengthOfUdpPacket = lengthOfUdpPacket * 10 + numberTmp - '0';
          }
          else //data maybe bit-error
          {
            sim_buff_index = pointerTo_r_n - sim_buff + 2; // +2 since "\r\n"
            continue;
          }
          posOfSubStrSave++;
        }
        posOfSubStrSave += 2;                                                 //point to data
        if (posOfSubStrSave + lengthOfUdpPacket > sim_buff + sim_buff_length) // don't have enough data
          return sim_buff_index;
        //else data is sastified
        sim7600_handle_udp_packet(posOfSubStrSave, lengthOfUdpPacket);
        sim_buff_index = posOfSubStrSave - sim_buff + lengthOfUdpPacket;
        continue;
      }
      else if (posOfSubStrSave + 6 < sim_buff + sim_buff_length) //the worst case: +RECEIVE,1,9999\r\n
      {
        //data may be error-bit
        sim_buff_index = posOfSubStrSave - sim_buff + 6;
        continue;
      }
      else
        return sim_buff_index; //dont have enough data
    }
    else if (resultCheck == 1) //1 net error
    {
      resultTmp = check_normal_response(posOfSubStrSave, listResponse[resultCheck], &sim_buff_index);
      if(resultTmp == 0) // success find
      {
        //sim7600 error, handle in a task sim7600 config task
        sim7600_network_IsOpen = false;
        sim7600_udp_IsOpen = false;
        sim7600_tcp_IsOpen = false;
        sim7600_error = true;
        continue;
      }
    }
    else if (resultCheck == 2) //2 udp closed
    {
      resultTmp = check_normal_response(posOfSubStrSave, listResponse[resultCheck], &sim_buff_index);
      if(resultTmp == 0) // success find
      {
        //sim7600 error, handle in a task sim7600 config task
        sim7600_udp_IsOpen = false;
        sim7600_error = true;
        continue;
      }
    }
    else if (resultCheck == 3) //3 can't open udp since network
    {
      resultTmp = check_normal_response(posOfSubStrSave, listResponse[resultCheck], &sim_buff_index);
      if(resultTmp == 0) // success find
      {
        //sim7600 error, handle in a task sim7600 config task
        sim7600_network_IsOpen = false;
        sim7600_udp_IsOpen = false;
        sim7600_error = true;
        continue;
      }
    }
    else if (resultCheck == 4) //4 call
    {
      resultTmp = check_normal_response(posOfSubStrSave, listResponse[resultCheck], &sim_buff_index);
      if(resultTmp == 0) // success find
      {
        sim7600_have_call = true;
        continue;
      }
    }
    else if (resultCheck == 5) //sms
    {
      resultTmp = check_normal_response(posOfSubStrSave, listResponse[resultCheck], &sim_buff_index);
      if(resultTmp == 0) // success find
      {
        //reserved
        continue;
      }
    }
    else if (resultCheck == 6) //"\r\n>""
    {
      if (sim7600_send_packetStatus1 == 1)
      {
        sim7600_send_packetStatus1 = 0;
        sim_buff_index = posOfSubStrSave - sim_buff + 3; // +1 point right after "\r\n>"
        continue;
      }
    }
    else //cmd check
    {
      //check whether have \r\n at the end of this response
      posOfSubStrSave += checkCmdResLength; //point to right after the last charater of response
      uint8_t* pointerTo_r_n = strstr(posOfSubStrSave, "\r\n");
      if(pointerTo_r_n != NULL)
      {
        if (cmdSendStatus == 1)
          cmdSendStatus = 0;
        else if (sim7600_open_tcp_connectStatus == 1)
          sim7600_open_tcp_connectStatus = 0;
        else if (sim7600_open_udp_connectStatus == 1)
          sim7600_open_udp_connectStatus = 0;
        else if (sim7600_open_netStatus == 1)
          sim7600_open_netStatus = 0;
        else if (sim7600_send_packetStatus2 == 1)
          sim7600_send_packetStatus2 = 0;
        res1 = NULL;
        res2 = NULL;
        //change sim_buff_index
        sim_buff_index = pointerTo_r_n - sim_buff + 2; // + 2 for "\r\n"
        continue;
      }
      else if(posOfSubStrSave + max_data_length_of_response_r_n < sim_buff + sim_buff_length) //the worst case
      {
        //data may be error-bit
        sim_buff_index = posOfSubStrSave - sim_buff + max_data_length_of_response_r_n;
        continue;
      }
      else //don't have enough data
        return sim_buff_index;
    }

    //data maybe bit-error sim_buff_index = posOfSubStrSave - sim_buff + max_data_length_of_response_r_n;
    if(resultTmp == 1) continue;
    //dont have enough data
    //else if(resultTmp == 2) return sim_buff_index;
    else return sim_buff_index;

    break;
  }

  //discard garbage \r\n .. \r\n
  while(true)
  {
    if (sim_buff_index >= sim_buff_length)
      return sim_buff_length;

    posOfSubStr = strstr(sim_buff + sim_buff_index, "\r\n"); //first \r\n
    if(posOfSubStr != NULL)
    {
      posOfSubStr = strstr(posOfSubStr + 2, "\r\n"); //second \r\n
      if(posOfSubStr != NULL) //detect \r\n...\r\n
      {
        sim_buff_index = posOfSubStr + 2 - sim_buff;
        continue;
      }
    }
    break; //if can't find \r\n .. \r\n sastified
  }
  return sim_buff_index;
}

int error_frame = 0;
int miss_frame = 0;
int late_frame = 0;
int frame_out_of_order = 0;
int conSotLai = 0;

int songID = 0;
int IDframePlayed = 0;
int IDframeSaved = 0;

int miss_adu = 0;
bool IsPlaying = false;

int totalFrame = 0;

void sim7600_handle_udp_packet(uint8_t* udpPacket, int length)
{
  //this var is maybe changed unexpected, so delcare  "a volatile pointer to a volatile variable"
  packetMP3HeaderStruct volatile * volatile packetMP3Header = (packetMP3HeaderStruct*)udpPacket;
  //check sum to confirm it is head of a packet
  if (packetMP3Header->checkSumHeader == sim7600_check_sum_data(udpPacket + 2, length - 2))
  {
    numOfPacketUDPReceived++;

    if (packetMP3Header->songID != songID) //new song
    {          
      //********* critical part **************
      //osThreadSuspendAll();
      osSemaphoreWait(BinSemPlayMp3Handle, osWaitForever);

      VS1003_ResetDecodeTime();

    	//clear status frame -> to detect old frame when play
    	for(int i = 0; i < mp3PacketSize; i++) 
      {
        mp3Packet[i].IsEmpty = true;
        mp3Packet[i].ID = 0;
      }

      IsPlaying = true;

      songID = packetMP3Header->songID; 

      IDframeSaved = 0; //debug

      totalFrame = 0;

      IDframePlayed = packetMP3Header->IDframe - 1; //to synchronize

      osSemaphoreRelease(BinSemPlayMp3Handle);
      //osThreadResumeAll();
    //****************************************************

      //put to task play mp3
      osMessagePut(play_mp3_queue_id, packetMP3Header->IDframe, osWaitForever); //wait before play
    }

    if(!IsPlaying) //new play after loss alot of packets
    {
      //********* critical part **************
      //osThreadSuspendAll();
      osSemaphoreWait(BinSemPlayMp3Handle, osWaitForever);


      //clear status frame -> to detect old frame when play
    	for(int i = 0; i < mp3PacketSize; i++) 
      {
        if(!mp3Packet[i].IsEmpty) conSotLai++;
        mp3Packet[i].IsEmpty = true;
        mp3Packet[i].ID = 0;
      }
      totalFrame = 0;
      
      IsPlaying = true;
      IDframePlayed = packetMP3Header->IDframe - 1; //to synchronize

      osSemaphoreRelease(BinSemPlayMp3Handle);
      //osThreadResumeAll();
    //****************************************************

      //put to task play mp3
      osMessagePut(play_mp3_queue_id, packetMP3Header->IDframe, osWaitForever); //wait before play
    }

    
    //debug
    if (packetMP3Header->IDframe > IDframeSaved)
    {
      miss_frame += packetMP3Header->IDframe - IDframeSaved - 1;
      IDframeSaved = packetMP3Header->IDframe;
    }
    else
    {
      late_frame++;
    }

    int index_mp3_packet = packetMP3Header->IDframe % mp3PacketSize;

    if(mp3Packet[index_mp3_packet].IsEmpty && packetMP3Header->IDframe > IDframePlayed)
    {
      //********* critical part **************
      //osThreadSuspendAll();
      osSemaphoreWait(BinSemPlayMp3Handle, osWaitForever);

      //clear garbage
      int adu_length = length - packetMP3HeaderLength;
      memset(mp3Packet[index_mp3_packet].frames + adu_length, 0, mp3PacketFrameSize - adu_length);
      memcpy(mp3Packet[index_mp3_packet].frames, packetMP3Header->frame, adu_length);
      mp3Packet[index_mp3_packet].ID = packetMP3Header->IDframe;
      //change bitrate to 144kbps
      mp3Packet[index_mp3_packet].frames[2] &= 0x0F;
      mp3Packet[index_mp3_packet].frames[2] |= 0xD0; //0b1101 0000
      //clear backpoint of playbuff
      mp3Packet[index_mp3_packet].frames[4] = 0;

      mp3Packet[index_mp3_packet].IsEmpty = false;

      totalFrame++;

      osSemaphoreRelease(BinSemPlayMp3Handle);
      //osThreadResumeAll();
      //****************************************************
    }
    else
    {
      frame_out_of_order++;
    }
  }
  else
  {
	  error_frame++; //debug
  }
}

int differ;

//sign: 1 ~ beggin play with delay n frame , 2 ~ play immidiate frame after dma tc interrupt
uint8_t playBuff[mp3PacketFrameSize];
void playMp3DMA(int IDframeWillplay)
{
  static int playMp3PacketIndex = 0;
  if(IDframeWillplay > 0)
  {
    //********* critical part **************
    //osThreadSuspendAll();
    osSemaphoreWait(BinSemPlayMp3Handle, osWaitForever);
    
    xQueueReset(play_mp3_queue_id);
    playMp3PacketIndex = IDframeWillplay % mp3PacketSize;

    osSemaphoreRelease(BinSemPlayMp3Handle);
    //osThreadResumeAll();
    //****************************************************

    osDelay(waitFrameBeforePlayNewSong * timePerFrame_ms); //8 frame delay
  }

  if(totalFrame  >= 20)
  {
	  totalFrame = 0;
  }
  if(IsPlaying)
  {
    //check whether play faster or slower synchronize time) use timer
    /*
    timer start when start new song (if(IDframeWillplay > 0))
    first iniatilize timer->CNT =  IDframePlayed
    */

    //********* critical part **************
    //osThreadSuspendAll();
    osSemaphoreWait(BinSemPlayMp3Handle, osWaitForever);

    if(totalFrame < 1)
    {
      totalFrame = 0;
      IsPlaying = false;
    }
    else
    {
      if (mp3Packet[playMp3PacketIndex].IsEmpty == false && mp3Packet[playMp3PacketIndex].ID > IDframePlayed)
      {
        memcpy(playBuff, mp3Packet[playMp3PacketIndex].frames, mp3PacketFrameSize);
        mp3Packet[playMp3PacketIndex].IsEmpty = true;
        IDframePlayed = mp3Packet[playMp3PacketIndex].ID;
        VS1003_Play_Data_DMA(playBuff, mp3PacketFrameSize);
        totalFrame--;
      }
      else
      {
        VS1003_Play_1frameMute_DMA();
        miss_adu++;
      }
      differ = IDframeSaved - IDframePlayed; //debug
      
      playMp3PacketIndex++;
      playMp3PacketIndex %= mp3PacketSize;
    }

    osSemaphoreRelease(BinSemPlayMp3Handle);
    //osThreadResumeAll();
    //****************************************************
  }
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

}

/**
 * \brief           usartSim7600 global interrupt handler
 */
int numOfIDLEdetect = 0;
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
    //
    //sim7600_pause_rx_uart_dma(0);

    //for safe, because dma handle with DR, so after disabel dma, we will read to clear IDLE
    //tmpreg = usartSim7600->DR;
    //(void)tmpreg;

    osMessagePut(usart_rx_dma_queue_id, 1, 0);

    numOfIDLEdetect++; //debug
  }
}

void sim7600_change_baud(uint32_t baudrate)
{
  LL_USART_Disable(USART1);
  sim7600SetBaudrate(baudrate);
  LL_USART_Enable(USART1);
}



//restart/reset module
//1st use pwr pin
//2nd use rst pin

void sim7600_restart()
{
  //suspend all task

  LL_USART_Disable(USART1);
  //disable DMA
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
  while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_2)); //wait until En bit == 0
  printf("%s", "rst\n");

  //int
  UDPsendStatus = 0;
  cmdSendStatus = 0;
  sim7600_open_tcp_connectStatus = 0;
  sim7600_open_udp_connectStatus = 0;
  sim7600_open_netStatus = 0;
  sim7600_send_packetStatus1 = 0;
  sim7600_send_packetStatus2 = 0;
  old_pos = 0;
  pos = 0;
  songID = 0;

  //bool
  Sim7600BasicConfigSuccess = false;
  sim7600_error = false;
  sim7600_network_IsOpen = false;
  sim7600_tcp_IsOpen = false;
  sim7600_udp_IsOpen = false;
  sim7600_have_call = false;
  IsPlaying = false;
  //init udp var
  // int i, limit = mp3PacketSize;
  // for(i = 0; i < limit; i++) mp3Packet[i].IsEmpty = true;

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

  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
	while(!LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_2)); //wait until En bit == 1

  //change baud rate to default
  sim7600_change_baud(Sim7600BaudDefaul); // in this function usart is enable again
  LL_USART_Enable(USART1);

  //power on again
  sim7600_powerON();

  //resume all task
}
