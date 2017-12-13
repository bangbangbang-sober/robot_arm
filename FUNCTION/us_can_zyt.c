
#include <stdio.h>
#include <string.h>
#include "us_mcu_transfer.h"
#include "us_can_zyt.h"

unsigned char us_uart_checksum(UART_INFO *recv_ptr)
{
	int ret = 0;
	int i = 0;
	unsigned int  sum = 0;

	if(recv_ptr->version == NULL  || recv_ptr->data == NULL || recv_ptr->length == 0){
		ret = -1;
	}
	if(!ret){

		sum += recv_ptr->type;
		sum += recv_ptr->length;
		for(i = 0; i < MAX_VERSION_LENGTH; i++){
			sum += recv_ptr->version[i];
		}
		
		for(i = 0; i < recv_ptr->length; i++){
			sum += recv_ptr->data[i];
		}
		sum = sum^0xFF;
		//DPRINTK("sum = %x\n", sum);
	}
	
	return sum;
}


unsigned char Usart_Buf[MAX_TRANS_INFO];
unsigned char Data_Cnt = 0;
unsigned long uart_id = 0;
US_DEV_TRANS trans_uart;

int us_usart_trans(void)
{
	int ret = 0;

	if(Data_Cnt != 0){
		trans_uart.cmd_id		= uart_id;
		trans_uart.dev			= USART_0;
		trans_uart.status		= 0;
		trans_uart.length		= Data_Cnt;
		//memset(trans_uart.data, 0, MAX_TRANS_INFO);
		memcpy(trans_uart.data, Usart_Buf, Data_Cnt);
		
		ret = us_mcu_rc_buff_enter(US_USART_READ, (unsigned char*)&trans_uart, sizeof(US_DEV_TRANS));

		uart_id++;
		if(uart_id >= 0xFFFFFF00){
			uart_id = 0;
		}
		
		Data_Cnt = 0;
	}

	return ret;
}
unsigned long long usart_rece_time = 0;
extern unsigned long long timer;

int us_usart_receivedata(unsigned char data)
{
	int ret = 0;
	
	//while(usart_ch != 0);
	//usart_ch = 1;
	
	Usart_Buf[ Data_Cnt++ ] = data;
	if(Data_Cnt == 0){
		usart_rece_time = timer;
	}
	
	if(Data_Cnt == MAX_TRANS_INFO){
		us_usart_trans();
 	}
	
	//usart_ch = 0;

	return ret;
}


#if 0
CAN_DATA info;
CAR_INFO send;
int can_data_status = HEAD;
int can_data_num;

unsigned char us_zmyt_uart_checksum(unsigned char type, unsigned char * data, unsigned char length)
{
	int ret = 0;
	int i = 0;
	unsigned int  sum = 0;

	if(data == NULL || length == 0){
		ret = -1;
	}
	if(!ret){

		sum += type;
		sum += length;
		
		for(i = 0; i < length; i++){
			sum += data[i];
		}
		sum = sum^0xFF;
		//DPRINTK("sum = %x\n", sum);
	}
	
	return sum;
}

unsigned char pakg = 0;
int us_zmyt_can_dcoder(void)
{
	int ret = 0;
	
	switch(info.type){
		case CARS_INFO:		
			send.type = US_BACK_CAR;
			if(((info.data[0] & 0x30) >> 4) == 0x1){//back
				send.data[0] = 0x00;
			}else if(((info.data[0] & 0x30) >> 4) == 0x2){
				send.data[0] = 0x01;
			}
			send.time = pakg;
			break;
		case TIRE_CORNER_INFO:		
			send.type = US_TIRE_CORNER_INFO;

			send.data[0] = info.data[0];
			send.time = pakg;
			
			break;

		default:
			
			break;
	}

	pakg++;
	
	return ret;
}


int can_receivedata( unsigned char data )
{
	int ret = 0;
	CAR_INFO *send_ptr = &send;

	switch(can_data_status){
		case HEAD:
			if(data == HEAD_CODE){
				info.head = data; 
				can_data_status = TYPE;
			}
	
			break;
		case TYPE:
			if(data < DECODER_CFG || data > DECODER_VERSION_INFO){
				info.head = 0;
				can_data_status = HEAD;
				ret = -1;
			}else{
				info.type = data;
				can_data_status = LENGTH;
			}
			break;
		case LENGTH:
			if(data <= 0 || data > MAX_CAN_DATA_LENGTH){
				info.head = 0;
				info.type = 0;
				can_data_status = HEAD;
				ret = -2;
			}else{
				info.length = data;
				can_data_num = 0;
				can_data_status = DATA_CAN;
			}
			
			break;
		case DATA_CAN:
			info.data[can_data_num] = data;
			can_data_num++;
			if(can_data_num == info.length){
				can_data_num = 0;
				can_data_status = CHECKSUM;
			}
			break;
		case CHECKSUM:
			if(data == us_zmyt_uart_checksum(info.type, info.data, info.length)){
				//memset(&info, 0, sizeof(CAN_INFO));	//clean the struct
				info.checksum = data;
				

				us_zmyt_can_dcoder();
				us_mcu_rc_buff_enter(US_CAN_DATA, (unsigned char*)send_ptr, sizeof(CAR_INFO));
				
				can_data_status = HEAD;
			}else{
				can_data_status = HEAD;
				ret = -3;
			}
			
			break;
		default:
			ret = -4;
			break;
	}
	
	
	return ret;
}
#endif

