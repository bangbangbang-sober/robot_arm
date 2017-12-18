#include<string.h>

#include "BSP_init.h"
#include "us_mcu_transfer.h"
#include "us_can_zyt.h"
#include "timer_function.h"
#include "iic_mpu9250.h"
#include "encoder.h"

#include "odometry.h"


#include "usbio.h"
#include "us_can_zyt.h"
#include "us_mcu_transfer.h"

/***************************************************************************************************/

extern s32 hSpeed_Buffer1[],hSpeed_Buffer2[];							//左右轮速度缓存数组
extern u8 main_sta;																				//主函数步骤执行标志位

//extern u8 bSpeed_Buffer_Index;
u8 bSpeed_Buffer_Index = 0;										 						//缓存左右轮编码数到数组变量

//extern float Milemeter_L_Motor,Milemeter_R_Motor;      	//累计电机一次运行的里程 cm		
float  Milemeter_L_Motor=0,Milemeter_R_Motor=0;						//dt时间内的左右轮速度,用于里程计计算

/***************************************************************************************************/

extern unsigned char DEV_STATUS_CTRL[US_DEV_NUM];
extern unsigned char US_MCU_STATUS;

int SENSOR_TIMESPS = 4;

extern US_HOST_PING ping_s;
extern unsigned long long ping_time;

unsigned long long timer = 0;
unsigned int SENSOR_PLAY = 0;

extern unsigned long long usart_rece_time;

void us_mcu_link_check(void)
{
	if((timer - ping_time < 80) && ping_s.link == LINK_OK){
		US_MCU_STATUS = MCU_DEV_LINK;
	}else{
		ping_time = timer;
		US_MCU_STATUS = MCU_DEV_UNLINK;
	}

	if(US_MCU_STATUS == MCU_DEV_UNLINK){
		DAC_SetChannel1Data(DAC_Align_12b_R, 2000);
		DAC_SetChannel2Data(DAC_Align_12b_R, 2000);
		DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);
		DAC_SoftwareTriggerCmd(DAC_Channel_2,ENABLE);
		
	}
	return ;
}

void us_mcu_ping(void)
{
	if(timer % 40 == 0){
		ping_s.mcu_status = US_MCU_STATUS;
		us_mcu_rc_buff_enter(US_MCU_PING, (unsigned char*)&ping_s, sizeof(US_HOST_PING));
	}	
	return;
}

void us_mcu_usart_timeout(void)
{
	if(timer - usart_rece_time >= 1){
		
		if(DEV_STATUS_CTRL[USART_0] == DEV_ON && US_MCU_STATUS == MCU_DEV_LINK){
			us_usart_trans();
		}
	}	
}

void us_mcu_g_sensor_timeout(void)
{
	if(timer % SENSOR_TIMESPS == 0){
		if(SENSOR_PLAY == 1){
			GetMPU9250Data();
		}
	}	
}

void TIM2_IRQHandler(void)
{

	if( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{
		//US_MCU_STATUS = MCU_DEV_LINK;
		us_mcu_link_check();
		us_mcu_ping();					//US MCU Ping
		us_mcu_g_sensor_timeout();
		//us_mcu_usart_timeout();		//Usart Timeout
		timer++;						//Tick
		
		TIM_ClearITPendingBit(TIM2 , TIM_IT_Update);
	}
}  


/*
//int temp = 0, count = 0, predir = 0, upcount = 0;
s32 Rcnt_L;
void TIM3_IRQHandler(void)
{
//	temp = (TIM_GetCounter(TIM3) & 0xffff);   
//	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){    
//		if(temp == 600*4){    
//			count--;
//			if(predir==0)//只有当前一次是负向走，这一次还是负向走才上传数据
//			{
//				upcount--;
//				//sjs_robot_encoder_send(SJS_ROBOT_ENCODER_L, "Left Encoder Data", 18, upcount);

//			}else{
//				predir=0;//表示往负向走      
//			}
//		}else if(temp == 0){
//			count++;
//			if(predir==1)//只有当前一次是正向走，这次又是正向走才上传数据
//			{
//				upcount++;
//				
//				//sjs_robot_encoder_send(SJS_ROBOT_ENCODER_L, "Left Encoder Data", 19, upcount);
//			}else{
//				predir = 1;//表示往正向走
//			}
//		}

//	}

	 if (TIM3 -> CR1 & 0X0010)		  //小心处理 溢出
   {
    Rcnt_L -= 1;
   }
   else 
    Rcnt_L += 1;

	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	
}

int temp_r = 0, count_r = 0, predir_r = 0, upcount_r = 0;
void TIM4_IRQHandler(void)
{
	temp_r=(TIM_GetCounter(TIM4)&0xffff);   

	if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){    
		
		if(temp_r == 600*4){    
		
			count_r--;

			if(predir_r==0)//只有当前一次是负向走，这一次还是负向走才上传数据
			{
				upcount_r--;
				
				//sjs_robot_encoder_send(SJS_ROBOT_ENCODER_R, "Right Encoder Data", 19, upcount_r);
			}else{
				
				predir_r=0;//表示往负向走      
			}
			
		}else if(temp_r==0){
			count_r++;
			if(predir_r==1)//只有当前一次是正向走，这次又是正向走才上传数据
			{
				upcount_r++;
				
				//sjs_robot_encoder_send(SJS_ROBOT_ENCODER_R, "Right Encoder Data", 19, upcount_r);
			}else{
				predir_r=1;//表示往正向走
			}
		}

	}    


	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}
*/
int ROBOT_ENCODER_EN = 0;
extern float pi;          //π
extern float position_x,position_y,oriention;         //计算得到位置坐标和方向角
void TIM5_IRQHandler(void)
{	
	//young
	if( TIM_GetITStatus(TIM5 , TIM_IT_Update) != RESET ) 
	{
//		odometry(Milemeter_R_Motor,Milemeter_L_Motor);//计算里程计
		if(ROBOT_ENCODER_EN == 1){
			sjs_robot_encoder_send(SJS_ROBOT_ENCODER, (int)position_x , (int)position_y, (int)((oriention * (180/pi))*100), timer);			
		}

		TIM_ClearITPendingBit(TIM5 , TIM_IT_Update);
	}
	
//	TIM_ClearITPendingBit(TIM5 , TIM_IT_Update);
}



void TIM6_IRQHandler(void)
{	
	//young
	if( TIM_GetITStatus(TIM6 , TIM_IT_Update) != RESET ) 
	{

#if 1
		if (hSpeedMeas_Timebase_500us !=0)//电机编码数采集时间间隔未到
		{
				hSpeedMeas_Timebase_500us--;//开始倒数	
		}
		else    //电机编码数采集时间间隔到了
		{
				
				s32 wtemp2,wtemp1;
				hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;//恢复电机编码数采集时间间隔
				
				/************************ 1 ***************************/
				
				wtemp2 = ENC_Calc_Rot_Speed2(); //A 获取的编码数
				wtemp1 = ENC_Calc_Rot_Speed1(); //B 获取的编码数
			
				/************************ 2 ***************************/
				
				//储存编码数（脉冲数），用于里程计计算
				Milemeter_L_Motor= (float)wtemp1; //储存脉冲数
				Milemeter_R_Motor= (float)wtemp2;
				
			  odometry(Milemeter_R_Motor,Milemeter_L_Motor);//计算里程计
			
//				printf("wtemp2 = %d , wtemp1 = %d \r\n",wtemp2, wtemp1);
				
				main_sta|=0x02;//执行计算里程计数据步骤

				/************************ 3 ***************************/
				
				//开始缓存左右轮编码数到数组
				hSpeed_Buffer2[bSpeed_Buffer_Index] = wtemp2;
				hSpeed_Buffer1[bSpeed_Buffer_Index] = wtemp1;
				bSpeed_Buffer_Index++;//数组移位
				
				//缓存左右轮编码数到数组结束判断
				if(bSpeed_Buffer_Index >=SPEED_BUFFER_SIZE)
				{
					bSpeed_Buffer_Index=0;//缓存左右轮编码数到数组变量清零
				}
				
				/************************ 4 ***************************/
				
				ENC_Calc_Average_Speed();//计算三次电机的平均编码数
				Gain2(); //电机A转速PID调节控制 右
				Gain1(); //电机B转速PID调节控制 左
    }
 #endif      
		TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);//清除中断标志位    
	}
}



