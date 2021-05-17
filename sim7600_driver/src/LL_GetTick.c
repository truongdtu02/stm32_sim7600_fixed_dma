/*
 * LL_GetTick.c
 *
 *  Created on: May 8, 2021
 *      Author: Bom
 */

#include "LL_GetTick.h"
uint64_t totalTime_ms_LL_GetTick = 0; //64-bit make sure that never overflow
//should set priority of timer interrupt is low
void Init_LL_GetTick()
{
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);

  /* TIM14 interrupt Init */
  NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  TIM_InitStruct.Prescaler = 41999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM14, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM14);
  //LL_TIM_EnableARRPreload(TIM14);
  TIM14->DIER |= 1; // enable interrupt
  TIM14->SR = 0; // clear flag
  LL_TIM_EnableCounter(TIM14);

}

//handle interrupt, place this function on IRQ handler function of timer
void IRQ_Tim_LL_GetTick()
{
    //check UIF: Update interrupt flag, UIE: Update interrupt enable
    if((TIM14->SR & 1) && (TIM14->DIER & 1))
    {
        //clear flag
        TIM14->SR &= ~1;
        totalTime_ms_LL_GetTick += 32768; // += 2^16 * 0.5 ms
        //LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_6);
        //printf("hello\r\n");
    }
    
}

uint64_t LL_GetTick_ms()
{
    volatile uint64_t tmpCNT = TIM14->CNT;
    tmpCNT = tmpCNT >> 1; // ~ / 2
    return totalTime_ms_LL_GetTick + tmpCNT;
}
