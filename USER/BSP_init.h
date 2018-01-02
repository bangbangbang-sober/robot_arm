#ifndef _BSP_H_
#define _BSP_H_

#define STM32_V4


/* ����Ƿ����˿������ͺ� */
#if !defined (STM32_V4) && !defined (STM32_X2)
	#error "Please define the board model : STM32_X2 or STM32_V4"
#endif

/* ���� BSP �汾�� */
#define __STM32F1_BSP_VERSION		"1.1"

/* CPU����ʱִ�еĺ��� */
//#define CPU_IDLE()		bsp_Idle()

/* ����ȫ���жϵĺ� */
#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */

#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"

#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

void Set_System(void);

/* �ṩ������C�ļ����õĺ��� */
void BSP_Configuration(void);
void bsp_Idle(void);
	
	
int SJS_MARM_Gpio_Init(void);
void Get_SerialNum(void);
void USB_Cable_Config (FunctionalState NewState);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void GPIO_Configuration(void);

int us_init_timer5(uint16_t freq);
	

#endif

