#ifndef __IMU_URANUS_H
#define __IMU_URANUS_H

#include "stm32f10x.h"

static u8 rev_buf[64]; /* ״̬������ */

enum input_status
{
	STATUS_IDLE, /*����*/
	STATUS_SOF,  /*���յ�֡ͷ (0x88, 0xAF)*/
	STATUS_LEN,  /*���յ������ֽ�*/
	STATUS_DATA, /*���յ�����*/
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

