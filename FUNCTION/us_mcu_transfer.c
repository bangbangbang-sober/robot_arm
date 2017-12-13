#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "usb_lib.h"

#include "us_mcu_transfer.h"
#include "us_can_zyt.h"
#include "usb_pwr.h"

#include "usart.h"
#include "IIC_MPU9250.h"



unsigned char MCU_VERSION[8] = "V1.1";


extern unsigned long long timer;

/*Ping*/
unsigned long long ping_time = 0;
extern US_HOST_PING ping_s;
US_HOST_PING ping_s;

UART_INFO rc_buf[MAX_BUFF_SIZE];

int front, rear;

unsigned char DEV_STATUS_CTRL[US_DEV_NUM];
unsigned char US_MCU_STATUS = MCU_DEV_LINK; 

unsigned char coder_id[20];
unsigned char coder_length;

unsigned int mcu_id_en[12] = {0};

extern unsigned int SENSOR_PLAY;
int u_front = 0;
int u_rear = 0;


extern unsigned char USART_SEND_BUFF[USART_MAX_BUFF_SEND];
extern int usart_lendth;

void cp_mcu_delay_ms(u16 time)
{
	u16 i=0;
	while(time--){
		i=12000;
		while(i--){}
	}
}

int us_mcu_rc_buff_init(void)
{
	front = 0;
	rear = 0;
	
	return OK;
}

int us_mcu_uart_coder(UART_INFO *send)
{
	int ret = 0;

	if(send == NULL){
		ret = -1;
	}
	if(!ret){
		send->head = UART_HEAD;
		memset((char *)send->version, 0, MAX_VERSION_LENGTH);
		sprintf((char *)send->version, "%s", ping_s.id);
		if(strlen((char *)send->version) >= MAX_VERSION_LENGTH - 1){
			ret = -2;
		}
	}
	if(!ret){
		send->checksum = us_uart_checksum(send);
	}

	if(ret < 0){
		us_dev_error(MCU_CONFIG, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}

	return 16 + send->length;
}


int us_dev_error(unsigned char dev, unsigned char* func_name, int name_len, int status)
{
	int ret = 0;
	US_DEV_TRANS trans_err = {0};

	if(func_name == NULL || name_len == 0){
		ret = -1;
	}
	if(!ret){
		memcpy(trans_err.data, func_name, name_len);
	}
	if(!ret){
		trans_err.cmd_id		= status;
		trans_err.dev			= dev;
		us_mcu_rc_buff_enter(US_MCU_ERR, (unsigned char*)&trans_err, sizeof(US_DEV_TRANS));
	}

	if(ret < 0){		
		//us_dev_error(MCU_CONFIG, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}

	return ret;
}


int sjs_robot_encoder_send(unsigned char cmd, int x, int y, int yaw, unsigned long long id)
{
	int ret = 0;
	US_DEV_TRANS trans_encoder = {0};
	SJS_MOVE_STATE move_state={0};
	//memset((char *)&motion_state, 0xff, sizeof(motion_state));

	if(!ret){
		move_state.id = id;
		move_state.x_position = x;
		move_state.y_position = y;
		move_state.yaw = yaw;
		memcpy(trans_encoder.data, (unsigned char *)&move_state, sizeof(SJS_MOVE_STATE));
		
		trans_encoder.length		= sizeof(SJS_MOVE_STATE);
	}
	
	if(!ret){
		trans_encoder.cmd_id		= 0;
		trans_encoder.dev				= cmd;
		us_mcu_rc_buff_enter(cmd, (unsigned char*)&trans_encoder, sizeof(US_DEV_TRANS));
	}

	if(ret < 0){		
		//us_dev_error(MCU_CONFIG, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}

	return ret;
}


int sjs_robot_ULT_send(unsigned char cmd, unsigned int left, unsigned int right, unsigned long long id)
{
	int ret = 0;
	US_DEV_TRANS trans_encoder = {0};
	SJS_ULT_STATE ULT_state = {0};
	//memset((char *)&motion_state, 0xff, sizeof(motion_state));

	if(!ret){
		ULT_state.id = id;
		ULT_state.left[0] = left;
		ULT_state.right[0] = right;
		memcpy(trans_encoder.data, (unsigned char *)&ULT_state, sizeof(SJS_ULT_STATE));
		
		trans_encoder.length		= sizeof(SJS_ULT_STATE);
	}
	if(!ret){
		trans_encoder.cmd_id		= 0;
		trans_encoder.dev			= cmd;
		us_mcu_rc_buff_enter(cmd, (unsigned char*)&trans_encoder, sizeof(US_DEV_TRANS));
	}

	if(ret < 0){		
		//us_dev_error(MCU_CONFIG, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}

	return ret;
}


int sjs_robot_usb_send(unsigned char cmd, unsigned char* data, int lenght, int status)
{
	int ret = 0;
	US_DEV_TRANS trans = {0};

	if(data == NULL || lenght == 0){
		ret = -1;
	}
	if(!ret){
		memcpy(trans.data, data, lenght);
	}
	if(!ret){
		trans.cmd_id		= status;
		trans.dev			= cmd;
		trans.length		= lenght;
		us_mcu_rc_buff_enter(cmd, (unsigned char*)&trans, sizeof(US_DEV_TRANS));
	}

	if(ret < 0){		
		//us_dev_error(MCU_CONFIG, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}

	return ret;
}

int us_usart_rc_buff_enter(unsigned char data)
{
	int ret = 0;

	if((u_rear + 1)%USART_MAX_BUFF_SEND == u_front){
		ret = -1;
	}

	if(!ret){
		USART_SEND_BUFF[u_rear] = data;
		u_rear = (u_rear + 1) % USART_MAX_BUFF_SEND;
	}

	if(ret < 0){
		us_dev_error(USART_0, (unsigned char *)__func__, strlen(__func__)+1, ret);
	}

	return ret;
}

int us_usart_rc_buff_delete(unsigned char *data)
{
	int ret = 0;

	if(data == NULL){
		ret = -1;
	}

	if(!ret){
		if( u_front == u_rear ){
			ret = -2;
		}
	}

	if(!ret){
		*data = USART_SEND_BUFF[u_front];
		u_front =(u_front + 1) % USART_MAX_BUFF_SEND;
	}
	
	return ret;
}

/****************************************************************/
int us_mcu_rc_buff_enter(unsigned char type, unsigned char *send, unsigned char buff_size)
{
	int ret = OK;
	
	if((rear + 1)%MAX_BUFF_SIZE == front){
		ret = -1;
	}
	
	if(!ret){
		rc_buf[rear].type = type;
		rc_buf[rear].length = buff_size;
		memcpy(rc_buf[rear].data, send, buff_size);
		rear = (rear + 1) % MAX_BUFF_SIZE;	
	}

	return ret;	
}

int us_mcu_rc_buff_delete(UART_INFO *str)
{
	int ret = OK;
	if(str == NULL){
		ret = -1;
	}
	if(!ret){
		if( front == rear ){
			ret = -2;  
		}
	}
	if(!ret){
		memcpy(str, &(rc_buf[front]), sizeof(UART_INFO));
		front =(front + 1) % MAX_BUFF_SIZE;
	}

	return ret;
}

int us_mcu_usb_send(unsigned char *send, unsigned int size)
{
	USB_SIL_Write(EP1_IN, send, size);

	return 0;
}


UART_INFO recv;
UART_INFO *recv_ptr 		= &recv;
unsigned char sdata[10] 	= {0};
unsigned int CpuID[4]		= {0};

#define STM32FLASH_EN_ID_START_ADDR		0x08032000//0x8020000//0x0807F800

int us_mcu_id_get(void)
{
	CpuID[0]=*(vu32*)(0x1ffff7e8);
	CpuID[1]=*(vu32*)(0x1ffff7ec);
	CpuID[2]=*(vu32*)(0x1ffff7f0);

	return 0;
}

int us_dev_trans_config(unsigned char type, US_DEV_TRANS *trans_buf)
{
	int ret = 0;
	unsigned char dev = trans_buf->dev;
	US_DEV_TRANS trans_send = {0};

	if(dev == MCU_DEV_CTRL){
		if(type == US_DEV_CFG_READ){
			memcpy(trans_send.data, DEV_STATUS_CTRL, US_DEV_NUM);
			trans_send.length		= US_DEV_NUM;		
			trans_send.status		= 0;
		}else if(dev == US_DEV_CFG_WRITE){
			memcpy(DEV_STATUS_CTRL, trans_send.data, US_DEV_NUM);
			trans_send.length		= 1;		
			trans_send.status		= 0;
		}
		
	}else{
		if(DEV_STATUS_CTRL[dev]	!= DEV_NULL){
			if(trans_buf->data[0] == DEV_OFF){
				DEV_STATUS_CTRL[dev] 	= 	trans_buf->data[0];
			}else if(trans_buf->data[0] == 	DEV_ON){
				DEV_STATUS_CTRL[dev] 	= 	trans_buf->data[0];
			}
			trans_send.status	= 0;
		}else{
			trans_send.status	= 5;
		}
		
		trans_send.length		= 1;
	}

	trans_send.cmd_id		= trans_buf->cmd_id;
	trans_send.dev			= trans_buf->dev;
	us_mcu_rc_buff_enter(type, (unsigned char*)&trans_send, sizeof(US_DEV_TRANS));


	return ret;
}

int us_mcu_input_data(unsigned char *input, unsigned int *length)
{
	int ret = 0, j = 0;
	unsigned char *input_data = input;
	unsigned int flash_RSA_code[12] = {0};

	if(input == NULL || length == NULL){
		ret = -1;
	}

	if(!ret){
		for(j = 0; j < 4; j++){
			flash_RSA_code[j] = *(u32*)(STM32FLASH_EN_ID_START_ADDR + j*4);
		}		
		
		memcpy(input_data, (unsigned char *)CpuID, MCU_ID_SIZE);
		memcpy(input_data + MCU_ID_SIZE, (unsigned char*)flash_RSA_code, RSA_KEY_BIT_SIZE/8);
		memcpy(input_data + MCU_ID_SIZE + RSA_KEY_BIT_SIZE/8, coder_id, TOKEN_SIZE);

		*length =  ENCODE_SIZE;
	}

	if(ret < 0){
		us_dev_error(MCU_CONFIG, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}	

	return ret;
}


#define  MY_PAGE_SIZE	0X800
//#define MY_PAGE_SIZE	PAGE_SIZE

unsigned int flash_get_data_size = 0;
unsigned char flash_cache[MY_PAGE_SIZE+100];
unsigned char *addr_ptr;

US_FLASH_TRANS flash_data;

int us_mcu_flash_load(unsigned char type, US_FLASH_TRANS *trans)
{
	int ret  = 0;
	
	addr_ptr = (unsigned char *)(MCU_FLASH_DATA_ADDR+trans->address);

	memcpy(flash_cache, addr_ptr, MY_PAGE_SIZE);

	return ret;
}

int us_mcu_flash_store(unsigned char type, US_FLASH_TRANS *trans)
{
	int ret  = 0, i = 0;

	FLASH_Unlock();

	FLASH_ErasePage(MCU_FLASH_DATA_ADDR+trans->address);

	for (i = 0; i < (MY_PAGE_SIZE /4); i++){
		if( FLASH_WaitForLastOperation(100000) != FLASH_TIMEOUT ){
			FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
		}
		FLASH_ProgramWord(MCU_FLASH_DATA_ADDR + trans->address + i * 4, *((unsigned int *)(flash_cache + i*4)));
	}
	
	FLASH_Lock();

	return ret;
}

unsigned long long FILL_CHECKSUM = 0;

int us_mcu_flash_fill(unsigned char type, US_FLASH_TRANS *trans)
{
	int ret  = 0;//, i = 0;
	unsigned int addr = 0;
	

	US_FLASH_TRANS *flash_trans = &flash_data;
	
	memcpy(flash_trans, trans, sizeof(US_FLASH_TRANS));
	
	if(flash_get_data_size == 0){
		//FILL_CHECKSUM = 0;
	}

	addr = (flash_trans->address)%MY_PAGE_SIZE;
	
	if(flash_get_data_size <  flash_trans->total_length){
		memcpy(flash_cache + addr + flash_get_data_size, flash_trans->data, flash_trans->send_length);
		flash_get_data_size += flash_trans->send_length;
	}

	if(flash_get_data_size == flash_trans->total_length){
		flash_get_data_size = 0;
		//for(i = 0 ; i < flash_trans->total_length ; i++){
		//	FILL_CHECKSUM+=flash_cache[i];
		//}
		//if(FILL_CHECKSUM != flash_trans->v_sum){
		//	us_dev_error(MCU_FLASH, (unsigned char *)__func__, strlen(__func__) + 1, 1);
		//} else {
		//	us_dev_error(MCU_FLASH, (unsigned char *)__func__, strlen(__func__) + 1, 0);
		//}
		//FILL_CHECKSUM = 0;
	}


	return ret;
}

unsigned long long FETCH_CHECKSUM = 0;
unsigned int fetch_send_length = 0;
int us_mcu_flash_fetch(unsigned char type, US_FLASH_TRANS *trans)
{
	int ret  = 0;
	int i = 0, num = 0;//, j = 0;
	US_FLASH_TRANS data = {0};
	unsigned char *addr_ptr = NULL;

	addr_ptr = flash_cache;
	
	data.total_length = trans->total_length;


	num = data.total_length/UPDATA_PKG_LENGTH;
	for(i = 0; i < num; i++){
		data.send_length = UPDATA_PKG_LENGTH;
		memcpy(data.data, addr_ptr + ( i * UPDATA_PKG_LENGTH ), UPDATA_PKG_LENGTH);
		//for(j = 0; j < data.send_length; j++){
		//	FETCH_CHECKSUM+=data.data[j];
		//}
		//data.v_sum = FETCH_CHECKSUM;
		
		us_mcu_rc_buff_enter(US_MCU_FLASH_READ, (unsigned char*)&data, sizeof(US_FLASH_TRANS));
		fetch_send_length+=UPDATA_PKG_LENGTH;
	}

	if(data.total_length % UPDATA_PKG_LENGTH != 0){
		data.send_length = data.total_length % UPDATA_PKG_LENGTH;
		memcpy(data.data, addr_ptr + ( i * UPDATA_PKG_LENGTH ), data.send_length);
		//for(j = 0; j < data.send_length; j++){
		//	FETCH_CHECKSUM+=data.data[j];
		//}
		//data.v_sum = FETCH_CHECKSUM;

		us_mcu_rc_buff_enter(US_MCU_FLASH_READ, (unsigned char*)&data, sizeof(US_FLASH_TRANS));
		fetch_send_length += data.send_length;
	}

	if(fetch_send_length == data.total_length){
		fetch_send_length = 0;
		//FETCH_CHECKSUM = 0;
	}
	
	return ret;
}

int us_dev_host_id_get(unsigned char *data, unsigned char length)
{
	int ret = 0;
	US_HOST_PING *ping_data = (US_HOST_PING *)data;
	
	if(length == sizeof(US_HOST_PING)){
		memcpy(ping_s.id, ping_data->id, ping_data->id_length);
		
		ping_s.id_length 	= ping_data->id_length;
		ping_s.link 		= ping_data->link;
		ping_s.host_status = ping_data->host_status;
		
		ping_time = timer;
	}

	return ret;
}


extern int SENSOR_TIMESPS;

int sns_mcu_sensor_decoder(unsigned char type, unsigned char *data)
{
	int ret  = 0;
	int timesps = 0;

	switch(type){
		case SNS_MCU_SENSOR_WRITE:
			if(data[0] == SENSOR_EN){
				SENSOR_PLAY = 1;
			}else if(data[0] == SENSOR_DIS){
				SENSOR_PLAY = 0;
			}
			break;
		case SNS_MCU_SENSOR_READ:
			GetMPU9250Data();
			break;
		case SNS_MCU_SENSOR_CONFIG:
			memcpy((char *)&timesps, data, 4);
			if((timesps > 0) && (timesps <=40)){
				SENSOR_TIMESPS = 1000/timesps/25;
			}
			
			break;
		default:
			break;
	}
		
	us_dev_error(type, (unsigned char *)__func__, strlen(__func__) + 1, SENSOR_TIMESPS);

	return ret;
}

extern int ROBOT_ENCODER_EN;

int robot_mcu_Encoder_decoder(unsigned char type, unsigned char *data)
{
	int ret  = 0;
	int timesps = 0;

	switch(type){
		case SJS_ROBOT_ENCODER_WRITE:
			if(data[0] == ENCODER_EN){
				ROBOT_ENCODER_EN = 1;
			}else if(data[0] == ENCODER_DIS){
				ROBOT_ENCODER_EN = 0;
			}
			break;
		case SJS_ROBOT_ENCODER_READ:
			
			break;
		case SJS_ROBOT_ENCODER_CONFIG:
			memcpy((char *)&timesps, data, 4);
			if((timesps > 0) && (timesps <=1000)){
				us_init_timer5(timesps);
			}
			
			break;
		default:
			break;
	}
		
	us_dev_error(type, (unsigned char *)__func__, strlen(__func__) + 1, ROBOT_ENCODER_EN);

	return ret;
}
int robot_mcu_pwr_decoder(unsigned char type, unsigned char *data)
{
	int ret = 0;
	
	if(data[0] == ROBOT_PWR_EN){
		GPIO_SetBits(GPIOA, GPIO_Pin_0);
		cp_mcu_delay_ms(400);
		GPIO_ResetBits(GPIOA, GPIO_Pin_0);
 	}else if(data[0] == ROBOT_PWR_DIS){
		GPIO_ResetBits(GPIOA, GPIO_Pin_0);
 	}

	return ret;
}

int robot_mcu_speed_decoder(unsigned char type, unsigned char *data)
{
	int ret = 0;
	
	if(data[0] == SPEED_UP){
		GPIO_SetBits(GPIOA, GPIO_Pin_1);
		cp_mcu_delay_ms(100);
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	}else if(data[0] == SPEED_DOWN){
		GPIO_SetBits(GPIOA, GPIO_Pin_2);
		cp_mcu_delay_ms(100);
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	}
	
	return ret;
}

int robot_mcu_whistled_decoder(unsigned char type, unsigned char *data)
{
	int ret = 0;
	
	if(data[0] == WHISTLE_ON){
		GPIO_SetBits(GPIOA, GPIO_Pin_3);
	}else if(data[0] == WHISTLE_OFF){
		GPIO_ResetBits(GPIOA, GPIO_Pin_3);
	}

	return ret;
}

int robot_mcu_joystick_decoder(unsigned char type, unsigned char *data)
{
	int ret = 0;
	ROBOT_JOYSTICK_TRANS trans = {0};

	
	memcpy((char *)&trans, data, sizeof(ROBOT_JOYSTICK_TRANS));

	if(US_MCU_STATUS == MCU_DEV_UNLINK){
		DAC_SetChannel1Data(DAC_Align_12b_R, 2000);
		DAC_SetChannel2Data(DAC_Align_12b_R, 2000);
		DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);
		DAC_SoftwareTriggerCmd(DAC_Channel_2,ENABLE);
	}else{
		DAC_SetChannel1Data(DAC_Align_12b_R, trans.x);
		DAC_SetChannel2Data(DAC_Align_12b_R, trans.y);
		DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);
		DAC_SoftwareTriggerCmd(DAC_Channel_2,ENABLE);
	}


	return ret;
}


extern uint8_t USB_Receive_Buffer[REPORT_COUNT];

int sjs_mcu_id_get(unsigned char type, US_DEV_TRANS *trans_buf)
{
	int ret = 0;
	US_MCU_ID_GET mcu_id_v;
	US_DEV_TRANS trans_send = {0};

	memcpy(mcu_id_v.MCU_Version, MCU_VERSION, 8);
	memcpy(mcu_id_v.CPUID, (unsigned char*)CpuID, MCU_ID_SIZE);
	

	if(*(u32*)(STM32FLASH_EN_ID_START_ADDR) != 0xffffffff){
		mcu_id_v.MCU_V[0] = '@';
	}else{
		mcu_id_v.MCU_V[0] = '#';
	};

	memcpy(trans_send.data, (unsigned char*)&mcu_id_v, sizeof(US_MCU_ID_GET));

	trans_send.cmd_id		= trans_buf->cmd_id;
	trans_send.dev		= trans_buf->dev;
	trans_send.status		= 0;
	
	trans_send.length = sizeof(US_MCU_ID_GET);
	us_mcu_rc_buff_enter(type, (unsigned char*)&trans_send, sizeof(US_DEV_TRANS));

	return ret;
}

void us_mcu_recave(void)
{
	unsigned char type = 0;
	
#ifndef STM32F10X_CL
	PMAToUserBufferCopy((unsigned char *)recv_ptr, ENDP1_RXADDR, REPORT_COUNT);
	SetEPRxStatus(ENDP1, EP_RX_VALID);
#else
	USB_SIL_Read(EP1_OUT,(unsigned char *)recv_ptr);
#endif

	if(1){
		if(recv_ptr->checksum == us_uart_checksum(recv_ptr)){
			type = recv_ptr->type;
			
			switch(type){
				case US_HOST_ID:
					us_dev_host_id_get(recv_ptr->data, recv_ptr->length);
					break;
				case US_MCU_ID_READ:
					sjs_mcu_id_get(type, (US_DEV_TRANS *)recv_ptr->data);
					break;
				#if 0
				case SNS_MCU_SENSOR_WRITE:
				case SNS_MCU_SENSOR_READ:
				case SNS_MCU_SENSOR_CONFIG:
					sns_mcu_sensor_decoder(type, recv_ptr->data);
					break;
				#endif
				case SJS_ROBOT_ENCODER_WRITE:
				case SJS_ROBOT_ENCODER_READ:
				case SJS_ROBOT_ENCODER_CONFIG:
					robot_mcu_Encoder_decoder(type, recv_ptr->data);
					break;
				case SJS_ROBOT_PWR:
					robot_mcu_pwr_decoder(type, recv_ptr->data);
					break;
				case SJS_ROBOT_SPEED:
					robot_mcu_speed_decoder(type, recv_ptr->data);
					break;
				case SJS_ROBOT_WHISTLED:
					robot_mcu_whistled_decoder(type, recv_ptr->data);				
					break;
				case SJS_ROBOT_JOYSTICK:
					robot_mcu_joystick_decoder(type, recv_ptr->data);
					break;
				case US_MCU_FLASH_LOAD:
					us_mcu_flash_load(type, (US_FLASH_TRANS *)recv_ptr->data);
					break;
				case US_MCU_FLASH_STORE:
					us_mcu_flash_store(type, (US_FLASH_TRANS *)recv_ptr->data);
					break;
				case US_MCU_FLASH_FILL:
					us_mcu_flash_fill(type, (US_FLASH_TRANS *)recv_ptr->data);
					break;	
				case US_MCU_FLASH_FETCH:
					us_mcu_flash_fetch(type, (US_FLASH_TRANS *)recv_ptr->data);
					break;
				
			};
			
		}else{
			us_dev_error(MCU_CONFIG, (unsigned char *)__func__, strlen(__func__) + 1, type);	
		}
	}

	return;
}



