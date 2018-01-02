
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

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�жϷ���2 ��λ��ռ(0-3) ��λ��Ӧ(0-3)
	
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

void TIM_Configuration(void)//TIM��ʼ��
{
//  TIM1_Configuration();			//PWM�����ʼ��
	TIM2_Configuration();			//��Ϣ�����ʼ��
	us_init_timer3(99);			//���õ��B TIM3������ģʽPA6 PA7 ����
//	TIM4_Configuration();			//���õ��A TIM4������ģʽPB6 PB7 �ҵ��
//	TIM6_Configuration();			//�ٶȼ��㶨ʱ����ʼ��
	
	
	us_init_timer5(14400);
}

void TIM1_Configuration(void)//TIM1��ͨ���ֱ����PWM1/PWM2,��Ӧ����Ϊ
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   //�������������Ϊ3.3v   
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    
	//����TIM1
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
	TIM_OCInitStructure.TIM_Pulse = 127; 													//������������127ʱ,��ƽ��������
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	   	//��������<127ʱ,PWMΪ�ߵ�ƽ

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);											//ʹ��ͨ��1 PA8
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);											//ʹ��ͨ��4 PA11			
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

	TIM_DeInit(TIM2);																		//��λ��ʱ��
	TIM_TimeBaseStructure.TIM_Period=50;								//��ʱ����ʼֵ	 2000 == 1s
	TIM_TimeBaseStructure.TIM_Prescaler=(36000 - 1);		//ʱ��Ԥ��Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	// ʱ�ӷָ�
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);			//��ʼ����ʱ����ֵ
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);								//�����ʱ���жϱ�־ 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);						//ʹ���ж�
	TIM_Cmd(TIM2,ENABLE); 															//����ʱ��
	
	return 0;  
}



int us_init_timer3(uint16_t freq)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

	//TIM3��1ͨ�����pwm���ο��Ƶ������ӦPC6
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
	/* PWM ģʽ���� */
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

	/* PWM ģʽ���� */
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

	/* PWM ģʽ���� */
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
//	TIM4->CCER|=1<<1;   //����

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

    //TIM5��1ͨ�����PWM���ο��Ƶ������ӦPA0
    GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;                 	
    GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    //����PWMƵ��
    TIM_TimeBaseStructure.TIM_Period 			=	freq;                   		
    TIM_TimeBaseStructure.TIM_Prescaler 		=	99;                   		 	
    TIM_TimeBaseStructure.TIM_ClockDivision 	= 	0x0;
    TIM_TimeBaseStructure.TIM_CounterMode 	= 	TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);         //ʱ����Ԫ����
    TIM_ClearFlag(TIM5,TIM_FLAG_Update);										//�����ʱ���жϱ�־
    TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);								//ʹ���ж�
                                                                      
    //�趨ռ�ձ�
    TIM_OCStructInit(& TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_OCMode 			= TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_Pulse 				= 1080;                      			//%50ռ�ձ�
    TIM_OCInitStructure.TIM_OutputState 		= TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity 			= TIM_OCPolarity_High;    
    TIM_OC1Init(TIM5, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

#if 1
    TIM_OCInitStructure.TIM_OCMode 			= TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_Pulse 				= 1080;                      			//%50ռ�ձ�
    TIM_OCInitStructure.TIM_OutputState 		= TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity 			= TIM_OCPolarity_High;    
    TIM_OC2Init(TIM5, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

    TIM_OCInitStructure.TIM_OCMode 			= TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_Pulse 				= 1080;                      			//%50ռ�ձ�
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

	TIM_DeInit(TIM6);																					//��λ��ʱ��
	TIM_TimeBaseStructure.TIM_Period = 4000;									//��ʱ����ʼֵ	 72000000/9/4000 = 2000Hz 
	TIM_TimeBaseStructure.TIM_Prescaler=(9-1);								//ʱ��Ԥ��Ƶ
	
//	TIM_TimeBaseStructure.TIM_Period = 2000;								//��ʱ����ʼֵ	 200 == 10Hz == 100ms
//	TIM_TimeBaseStructure.TIM_Prescaler=(36000 - 1);				//ʱ��Ԥ��Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;			//ʱ�ӷָ�
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//���ϼ���ģʽ
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);						//��ʼ����ʱ����ֵ
	TIM_ClearFlag(TIM6,TIM_FLAG_Update);											//�����ʱ���жϱ�־ 
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);									//ʹ���ж�
	TIM_Cmd(TIM6,ENABLE); 																		//����ʱ��
	
	return 0;  
}


