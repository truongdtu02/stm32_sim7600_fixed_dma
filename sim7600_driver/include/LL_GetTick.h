/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LL_GETTICK_H
#define __LL_GETTICK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

//tick need a timer operate at basic, just count up, no auto preload, have interrupt
//ex: tim6, has 16-bit counter reg (CNT), clk freq = 84Mhz, count up
//set prescaler reg(PSC) = (42000-1). -> CNT++ every 0.5ms (=1/(clk_freq/(PSC+1))) or (=(PSC+1) / clk_freq)
//when CNT over flow that mean 2^16*0.5ms elapsed (da troi qua)

//should set priority of timer interrupt is low
void Init_LL_GetTick();

//handle interrupt, place this function on IRQ handler function of timer
void IRQ_Tim_LL_GetTick();

uint64_t LL_GetTick_ms();

#ifdef __cplusplus
}
#endif

#endif /* __LL_GETTICK_H */
