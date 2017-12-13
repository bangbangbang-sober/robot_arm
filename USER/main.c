#include "string.h"
#include "BSP_init.h"

#include "contact.h"
#include "odometry.h"

#include "usb_lib.h"
#include "usbio.h"
#include "us_can_zyt.h"
#include "us_mcu_transfer.h"

UART_INFO send_buf;

/***********************************************  输出  *****************************************************************/

char odometry_data[21]={0};   //发送给串口的里程计数据数组

float odometry_right=0,odometry_left=0;//串口得到的左右轮速度

/***********************************************  输入  *****************************************************************/

extern float position_x,position_y,oriention,velocity_linear,velocity_angular;         //计算得到的里程计数值

extern u8 USART_RX_BUF[USART_REC_LEN];     //串口接收缓冲,最大USART_REC_LEN个字节.
extern u16 USART_RX_STA;                   //串口接收状态标记	

extern float Milemeter_L_Motor,Milemeter_R_Motor;     //dt时间内的左右轮速度,用于里程计计算

/***********************************************  变量  *****************************************************************/

u8 main_sta=0; 				


union recieveData  		//接收到的数据
{
	float d;    		 		//左右轮速度
	unsigned char data[4];
}leftdata,rightdata;  //接收的左右轮数据


union odometry  	 		//里程计数据共用体
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}x_data,y_data,theta_data,vel_linear,vel_angular;     
//要发布的里程计数据，分别为：X，Y方向移动的距离，当前角度，线速度，角速度

/****************************************************************************************************************/	

 int main(void)
 {	 

#if 0
	 u8 t=0;
	 u8 i=0,j=0,m=0;
#endif

		UART_INFO *send_ptr = &send_buf;
		uint8_t *send = (uint8_t *)send_ptr;
		int send_size = 0;
	 
	 BSP_Configuration();		//板级资源 初始化
	 us_mcu_id_get();				//US Get MCU ID
	 
	 while(1)
	 {  
			delay_ms(100);
			odometry(Milemeter_R_Motor,Milemeter_L_Motor);//计算里程计		 
		 
			if(GetEPTxStatus(ENDP2) == EP_TX_NAK){
				if(us_mcu_rc_buff_delete(send_ptr) == OK){
					if((send_size = us_mcu_uart_coder(send_ptr)) < 0){
							us_dev_error(MCU_CONFIG, (unsigned char *)__func__, strlen(__func__)+1, send_size);
					}else{
							USB_SendData(send, send_size);
					}
				}
			}

			
#if 0
			//执行发送里程计数据步骤
			if(main_sta&0x01)
			{
					//里程计数据获取
					x_data.odoemtry_float=position_x;			//单位mm
					y_data.odoemtry_float=position_y;			//单位mm
					theta_data.odoemtry_float=oriention;		//单位rad
					vel_linear.odoemtry_float=velocity_linear;	//单位mm/s
					vel_angular.odoemtry_float=velocity_angular;//单位rad/s
							
					//将所有里程计数据存到要发送的数组
					for(j=0;j<4;j++)
					{
						odometry_data[j]=x_data.odometry_char[j];
						odometry_data[j+4]=y_data.odometry_char[j];
						odometry_data[j+8]=theta_data.odometry_char[j];
						odometry_data[j+12]=vel_linear.odometry_char[j];
						odometry_data[j+16]=vel_angular.odometry_char[j];
					}
							
					odometry_data[20]='\n';//添加结束符
							
					//发送数据要串口
					for(i=0;i<21;i++)
					{
						USART_ClearFlag(USART1,USART_FLAG_TC);  //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题				
						USART_SendData(USART1,odometry_data[i]);//发送一个字节到串口	
						while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	//等待发送结束			
					}
							
					main_sta&=0xFE;//执行计算里程计数据步骤
			}
		

	
			//执行计算里程计数据步骤
			if(main_sta&0x02)
			{
				odometry(Milemeter_R_Motor,Milemeter_L_Motor);//计算里程计
				main_sta&=0xFD;//执行发送里程计数据步骤
			} 
		
			
	
			//当发送指令没有正确接收时
			if(main_sta&0x08)        
			{
					USART_ClearFlag(USART1,USART_FLAG_TC);  //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题
					for(m=0;m<3;m++)
					{
							USART_SendData(USART1,0x00);	
							while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
					}		
					USART_SendData(USART1,'\n');	
					while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	
					main_sta&=0xF7;
			}
			
			
		
			// 串口1接收函数
			if(USART_RX_STA&0x8000)  
			{			
				//接收左右轮速度
				for(t=0;t<4;t++)
				{
						rightdata.data[t]=USART_RX_BUF[t];
						leftdata.data[t]=USART_RX_BUF[t+4];
				}

				//储存左右轮速度
				odometry_right=rightdata.d;
				odometry_left=leftdata.d;

				USART_RX_STA=0;//清楚接收标志位
			}
		
			
			//将接收到的左右轮速度赋给小车
			car_control(rightdata.d,leftdata.d);
#endif			
	 }	 //end while
 }	//end main

