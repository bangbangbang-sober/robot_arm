#include <stdio.h>
#include <string.h>

#include "stm32f10x.h"
#include "pwm.h"

void PwmStop(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	TIM_Cmd(TIM1,DISABLE);
	TIM_DeInit(TIM1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_ResetBits(GPIOA, (GPIO_Pin_8 | GPIO_Pin_11));
}

void Pwm_Ch1_Disable(void)
{
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
}

void Pwm_Ch1_Enable(void)
{
	TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Enable);
}

void Pwm_Ch4_Disable(void)
{
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
}

void Pwm_Ch4_Enable(void)
{
	TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Enable);
}

