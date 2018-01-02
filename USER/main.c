#include "string.h"
#include "BSP_init.h"

#include "contact.h"
#include "odometry.h"

#include "usb_lib.h"
#include "usbio.h"
#include "us_can_zyt.h"
#include "us_mcu_transfer.h"

#include "imu_uranus.h"

UART_INFO send_buf;
extern unsigned char US_MCU_STATUS;
u8 main_sta=0; 

/*********************************************** imu data ***************************************************************/
extern imu_data_t imu_data_3,imu_data_5;


uint16_t OUTPUT_BUFFER[20]={1500,1800,2100,2400,2700,3000,3300,3600,3900,4095,
							4095,3900,3600,3300,3000,2700,2400,2100,1800,1500};

uint16_t OUTPUT_BUFFER2[20]={500,800,1000,100,500,800,1000,100,500,800,
							1000,100,500,800,1000,100,500,800,1000,100};

 int main(void)
 {	 
		UART_INFO *send_ptr = &send_buf;
		uint8_t *send = (uint8_t *)send_ptr;
		int send_size = 0;

		BSP_Configuration();			//板级资源 初始化
		us_mcu_id_get();				//US Get MCU ID

		US_MCU_STATUS = MCU_DEV_INIT;

	 while(1)
	 {
//			delay_ms(1000);
//			printf("imu_data_5--G:%d \r\n",imu_data_5.accl[2]);
//			printf("imu_data_3--P/R/Y/P:%d %d %d \r\n",imu_data_3.pitch/100, imu_data_3.roll/100, imu_data_3.yaw/10);
			printf("imu_data_5--P/R/Y:%d %d %d \r\n",imu_data_5.pitch/100, imu_data_5.roll/100, imu_data_5.yaw/10);
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
 
 #ifdef  USE_FULL_ASSERT
/**
  * @brief  报告在检查参数发生错误时的源文件名和错误行数
  * @param  file 源文件名
  * @param  line 错误所在行数
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* 用户可以增加自己的代码用于报告错误的文件名和所在行数,
       例如:printf("错误参数值: 文件名 %s 在 %d行\r\n", file, line) */

    /* 无限循环 */
	while (1)
	{
		us_dev_error(MCU_CONFIG, (unsigned char *)file, strlen(file)+1, line);
		sleep(1);
	}
}
#endif

