#ifndef __US_CAN_ZYT_H
#define __US_CAN_ZYT_H

#include "us_mcu_transfer.h"

#define MCU_TEST					0
#define SENSOR_TEST				1
#define MY_IIC						1

typedef enum __CAN_DATA_STATUS{
	HEAD		= 0,
	TYPE,
	LENGTH,
	DATA_CAN,
	CHECKSUM
}CAN_DATA_STATUS;


/*CMD*/
#define ACK							0XFF
#define NACK_NG						0xF0		//checksun NG
#define NACK_NS						0XF3		//Not Support
#define NACK_B						0XFC		//Busy

/*CAN HEAD*/
#define HEAD_CODE					0x2E

/*Slave -> Host*/
#define DECODER_CFG					0x08
#define DECODER_STA					0x0A
#define CARS_INFO					0x10
#define CARS_DYMC_INFO				0x16
#define TIRE_CORNER_INFO			0x18
#define RADAR_INFO					0x1A
#define R_KEY_INFO					0x20
#define DECODER_VERSION_INFO		0x30

/*Host -> Slave*/
#define START_END					0x81
#define DECODER_GET					0x90
#define DECODER_SET					0x98

typedef struct _CAN_DATA{
	unsigned char head;
	unsigned char type;
	unsigned char length;
	unsigned char checksum;
	unsigned char data[MAX_CAN_DATA_LENGTH];
}CAN_DATA;


unsigned char us_zmyt_uart_checksum(unsigned char type, unsigned char * data, unsigned char length);
int can_receivedata( unsigned char data );
int us_usart_receivedata(unsigned char data);
int us_usart_trans(void);

#endif
