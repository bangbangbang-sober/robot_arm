#ifndef _BSP_H_
#define _BSP_H_

#define STM32_V4


/* 检查是否定义了开发板型号 */
#if !defined (STM32_V4) && !defined (STM32_X2)
	#error "Please define the board model : STM32_X2 or STM32_V4"
#endif

/* 定义 BSP 版本号 */
#define __STM32F1_BSP_VERSION		"1.1"

/* CPU空闲时执行的函数 */
//#define CPU_IDLE()		bsp_Idle()

/* 开关全局中断的宏 */
#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */

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

/* 提供给其他C文件调用的函数 */
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

