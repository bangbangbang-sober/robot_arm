#include <stdio.h>

#include "imu_uranus.h"

imu_data_t imu_data_3,imu_data_5;

/*接收函数，这个函数应该在串口中断中被调用，将接受的字节传入ch*/
int imu_rev_process(u8 ch,u8 uart_num)
{
	static int len = 0;
	static int i; /*状态*/
	static enum input_status status = STATUS_IDLE; /* running status machine */
	switch(status)
	{
		case STATUS_IDLE:
			if((uint8_t)ch == 0x88){ 
				status = STATUS_SOF; 
			}
			break;
		case STATUS_SOF:
			if((uint8_t)ch == 0xAF){ 
				status = STATUS_LEN; 
			} 
			break;
		case STATUS_LEN:
			len = ch; 
			status = STATUS_DATA; 
			i = 0; 
			break;
		case STATUS_DATA:
			if(i == len){
				status = STATUS_IDLE; /* 完整的接受完一帧数据，调用解析函数*/
				if(uart_num == 3){
					imu_rev_get_data(&imu_data_3);
				}else if(uart_num == 5){
					imu_rev_get_data(&imu_data_5);
				}
				
				break;
			}
			rev_buf[i++] = ch; 
			break;
		default: 
			break;
	}
	return 0;
	
}

int imu_rev_get_data(imu_data_t *imu_data)
{
	imu_data->accl[0] = (rev_buf[0]<<8) + rev_buf[1];
	imu_data->accl[1] = (rev_buf[2]<<8) + rev_buf[3];
	imu_data->accl[2] = (rev_buf[4]<<8) + rev_buf[5];
	imu_data->gyro[0] = (rev_buf[6]<<8) + rev_buf[7];
	imu_data->gyro[1] = (rev_buf[8]<<8) + rev_buf[9];
	imu_data->gyro[2] = (rev_buf[10]<<8) + rev_buf[11];
	imu_data->mag[0] = (rev_buf[12]<<8) + rev_buf[13];
	imu_data->mag[1] = (rev_buf[14]<<8) + rev_buf[15];
	imu_data->mag[2] = (rev_buf[16]<<8) + rev_buf[17];
	imu_data->roll = (rev_buf[18]<<8) + rev_buf[19];
	imu_data->pitch = (rev_buf[20]<<8) + rev_buf[21];
	imu_data->yaw = (rev_buf[22]<<8) + rev_buf[23];
	imu_data->presure = (rev_buf[27]<<24) + (rev_buf[26]<<16) + (rev_buf[25]<<8) +(rev_buf[24]<<0);
	
//	printf("P/R/Y/P:%05d %05d %05d \r\n",imu_data->pitch/100, imu_data->roll/100, imu_data->yaw/10);
//	printf("acc->z:%05d \r",imu_data->accl[2]/100);
	return 0;
}


