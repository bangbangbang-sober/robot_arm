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
		UART_INFO *send_ptr = &send_buf;
		uint8_t *send = (uint8_t *)send_ptr;
		int send_size = 0;
	 
	 BSP_Configuration();		//板级资源 初始化
	 us_mcu_id_get();				//US Get MCU ID

	 while(1)
	 {  			
		 		if(GetEPTxStatus(ENDP2) == EP_TX_NAK){
					 if(us_mcu_rc_buff_delete(send_ptr) == OK){
							if((send_size = us_mcu_uart_coder(send_ptr)) < 0){
									us_dev_error(MCU_CONFIG, (unsigned char *)__func__, strlen(__func__)+1, send_size);
						 }else{
								 USB_SendData(send, send_size);
						 }
					 }
				 }				 
	 }	 //end while
 }	//end main

