
#include "timer_function.h"


void RCC_Configuration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure; 

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断分组2 两位抢占(0-3) 两位响应(0-3)
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);   

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}

void TIM_Configuration(void)//TIM初始化
{
//  TIM1_Configuration();			//PWM输出初始化
	TIM2_Configuration();			//信息传输初始化
	us_init_timer3(99);			//设置电机B TIM3编码器模式PA6 PA7 左电机
//	TIM4_Configuration();			//设置电机A TIM4编码器模式PB6 PB7 右电机
//	TIM6_Configuration();			//速度计算定时器初始化
	
	
	us_init_timer5(14400);
}

void TIM1_Configuration(void)//TIM1两通道分别输出PWM1/PWM2,对应引脚为
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   //复用满负载输出为3.3v   
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    
	//配置TIM1
	TIM_DeInit(TIM1); 
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseStructure.TIM_Period = 500;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	             
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 127; 													//当计数器到达127时,电平发送跳变
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	   	//当计数器<127时,PWM为高电平

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);											//使能通道1 PA8
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);											//使能通道4 PA11			
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
//	
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	
	TIM1->CCR1 = 127; 
	TIM1->CCR4 = 127;   

	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
}
////////////////////////////

int TIM2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	TIM_DeInit(TIM2);																		//复位定时器
	TIM_TimeBaseStructure.TIM_Period=50;								//定时器初始值	 2000 == 1s
	TIM_TimeBaseStructure.TIM_Prescaler=(36000 - 1);		//时钟预分频
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	// 时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//向上计数模式
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);			//初始化定时器的值
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);								//清除定时器中断标志 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);						//使能中断
	TIM_Cmd(TIM2,ENABLE); 															//开启时钟
	
	return 0;  
}



int us_init_timer3(uint16_t freq)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

	//TIM3的1通道输出pwm波形控制电机，对应PC6
	GPIO_InitStructure.GPIO_Pin 			= GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8; 
	GPIO_InitStructure.GPIO_Mode 			= GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 360;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
#if 1
	/* PWM 模式配置 */
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;//++
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//++
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;//++
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;//++
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* PWM 模式配置 */
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;//++
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//++
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;//++
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;//++
	
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);
#endif

	/* PWM 模式配置 */
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;//++
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//++
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;//++
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;//++
	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	//TIM_ITConfig(TIM3,TIM_IT_Update, ENABLE);
	
	TIM_ITConfig(TIM3,  TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM3,  TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM3,  TIM_IT_CC3, ENABLE);

	return 0;
}


int TIM4_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   	
	NVIC_InitTypeDef NVIC_InitStructure2;

	//PB6 ch1  A,PB7 ch2 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);                           


	NVIC_InitStructure2.NVIC_IRQChannel = TIM4_IRQn; 
	NVIC_InitStructure2.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure2.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure2.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure2);

	TIM_DeInit(TIM4);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 600 * 4 - 1; 
	TIM_TimeBaseStructure.TIM_Prescaler = 0; 
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);              
                 
	//TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6; 
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
//	//Reset counter
//	TIM4->CNT = 0;
//	TIM4->CCER|=1<<1;   //反向

	TIM_Cmd(TIM4, ENABLE); 

	return 0;
}

int us_init_timer5(uint16_t freq)
{
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	    
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 
  //  GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2 ,ENABLE ); 
  //  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

    //TIM5的1通道输出PWM波形控制电机，对应PA0
    GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;                 	
    GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    //定义PWM频率
    TIM_TimeBaseStructure.TIM_Period 			=	freq;                   		
    TIM_TimeBaseStructure.TIM_Prescaler 		=	99;                   		 	
    TIM_TimeBaseStructure.TIM_ClockDivision 	= 	0x0;
    TIM_TimeBaseStructure.TIM_CounterMode 	= 	TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);         //时基单元配置
    TIM_ClearFlag(TIM5,TIM_FLAG_Update);										//清楚定时器中断标志
    TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);								//使能中断
                                                                      
    //设定占空比
    TIM_OCStructInit(& TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_OCMode 			= TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_Pulse 				= 1080;                      			//%50占空比
    TIM_OCInitStructure.TIM_OutputState 		= TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity 			= TIM_OCPolarity_High;    
    TIM_OC1Init(TIM5, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

#if 1
    TIM_OCInitStructure.TIM_OCMode 			= TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_Pulse 				= 1080;                      			//%50占空比
    TIM_OCInitStructure.TIM_OutputState 		= TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity 			= TIM_OCPolarity_High;    
    TIM_OC2Init(TIM5, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

    TIM_OCInitStructure.TIM_OCMode 			= TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_Pulse 				= 1080;                      			//%50占空比
    TIM_OCInitStructure.TIM_OutputState 		= TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity 			= TIM_OCPolarity_High;    
    TIM_OC3Init(TIM5, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
#endif

	
    TIM_ARRPreloadConfig(TIM5,ENABLE);
    TIM_Cmd(TIM5, ENABLE);
    TIM_CtrlPWMOutputs(TIM5, ENABLE); 

	return 0;
}

int TIM6_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);

	TIM_DeInit(TIM6);																					//复位定时器
	TIM_TimeBaseStructure.TIM_Period = 4000;									//定时器初始值	 72000000/9/4000 = 2000Hz 
	TIM_TimeBaseStructure.TIM_Prescaler=(9-1);								//时钟预分频
	
//	TIM_TimeBaseStructure.TIM_Period = 2000;								//定时器初始值	 200 == 10Hz == 100ms
//	TIM_TimeBaseStructure.TIM_Prescaler=(36000 - 1);				//时钟预分频
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;			//时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//向上计数模式
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);						//初始化定时器的值
	TIM_ClearFlag(TIM6,TIM_FLAG_Update);											//清除定时器中断标志 
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);									//使能中断
	TIM_Cmd(TIM6,ENABLE); 																		//开启时钟
	
	return 0;  
}


