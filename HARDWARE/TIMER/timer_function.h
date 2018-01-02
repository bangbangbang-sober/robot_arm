#ifndef _TIMER_FUNCTION_H_
#define _TIMER_FUNCTION_H_

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"


void RCC_Configuration(void);
void NVIC_Configuration(void);


void TIM_Configuration(void);
void TIM1_Configuration(void);
int TIM2_Configuration(void);
int TIM3_Configuration(void);
int TIM4_Configuration(void);
int TIM6_Configuration(void);

int us_init_timer3(uint16_t freq);
int us_init_timer5(uint16_t freq);




#endif

