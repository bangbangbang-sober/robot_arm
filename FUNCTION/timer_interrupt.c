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

extern s32 hSpeed_Buffer1[],hSpeed_Buffer2[];							//�������ٶȻ�������
extern u8 main_sta;																				//����������ִ�б�־λ

//extern u8 bSpeed_Buffer_Index;
u8 bSpeed_Buffer_Index = 0;										 						//���������ֱ��������������

//extern float Milemeter_L_Motor,Milemeter_R_Motor;      	//�ۼƵ��һ�����е���� cm		
float  Milemeter_L_Motor=0,Milemeter_R_Motor=0;						//dtʱ���ڵ��������ٶ�,������̼Ƽ���

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

int ROBOT_ENCODER_EN = 0;
extern float pi;          //��
extern float position_x,position_y,oriention;         //����õ�λ������ͷ����
void TIM5_IRQHandler(void)
{	
	//young
	if( TIM_GetITStatus(TIM5 , TIM_IT_Update) != RESET ) 
	{
		if(ROBOT_ENCODER_EN == 1){
			sjs_robot_encoder_send(SJS_ROBOT_ENCODER, (int)position_x , (int)position_y, (int)((oriention * (180/pi))*100), timer);			
		}

		TIM_ClearITPendingBit(TIM5 , TIM_IT_Update);
	}
}



void TIM6_IRQHandler(void)
{	
	//young
	if( TIM_GetITStatus(TIM6 , TIM_IT_Update) != RESET ) 
	{

		if (hSpeedMeas_Timebase_500us !=0)	//����������ɼ�ʱ����δ��
		{
				hSpeedMeas_Timebase_500us--;		//��ʼ����	
		}
		else    														//����������ɼ�ʱ��������
		{
				
				s32 wtemp2,wtemp1;
				hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;//�ָ�����������ɼ�ʱ����
				
				/************************ 1 ***************************/
				
				wtemp2 = ENC_Calc_Rot_Speed2(); //A ��ȡ�ı�����
				wtemp1 = ENC_Calc_Rot_Speed1(); //B ��ȡ�ı�����
			
				/************************ 2 ***************************/
				
				//���������������������������̼Ƽ���
				Milemeter_L_Motor= (float)wtemp1; //����������
				Milemeter_R_Motor= (float)wtemp2;
				
			  odometry(Milemeter_R_Motor,Milemeter_L_Motor);//������̼�
				
				main_sta|=0x02;//ִ�м�����̼����ݲ���

				/************************ 3 ***************************/
				
				//��ʼ���������ֱ�����������
				hSpeed_Buffer2[bSpeed_Buffer_Index] = wtemp2;
				hSpeed_Buffer1[bSpeed_Buffer_Index] = wtemp1;
				bSpeed_Buffer_Index++;//������λ
				
				//���������ֱ���������������ж�
				if(bSpeed_Buffer_Index >=SPEED_BUFFER_SIZE)
				{
					bSpeed_Buffer_Index=0;//���������ֱ������������������
				}
				
				/************************ 4 ***************************/
				
				ENC_Calc_Average_Speed();//�������ε����ƽ��������
				Gain2(); //���Aת��PID���ڿ��� ��
				Gain1(); //���Bת��PID���ڿ��� ��
    }      
		TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);//����жϱ�־λ    
	}
}



