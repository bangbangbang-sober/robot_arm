#ifndef __IMU_URANUS_H
#define __IMU_URANUS_H

#include "stm32f10x.h"

static u8 rev_buf[64]; /* 状态机常量 */

enum input_status
{
	STATUS_IDLE, /*空闲*/
	STATUS_SOF,  /*接收到帧头 (0x88, 0xAF)*/
	STATUS_LEN,  /*接收到长度字节*/
	STATUS_DATA, /*接收到数据*/
};
typedef struct 
{
	int16_t accl[3];
	int16_t gyro[3];
	int16_t mag[3];
	uint16_t yaw;
	int16_t pitch;
	int16_t roll;
	int32_t presure;
}imu_data_t;

int imu_rev_get_data(imu_data_t *imu_data);
int imu_rev_process(u8 ch,u8 uart_num);


#endif

