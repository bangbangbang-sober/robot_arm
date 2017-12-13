#ifndef _CP_EXT_RC_BUFFER_H
#define _CP_EXT_RC_BUFFER_H

#include "stm32f10x.h"

#include "usb_desc.h"

#define ERROR 							-1
#define OK								0
#define MAX_BUFF_SIZE					150

/*Can info*/
//Define

#define MAX_CAN_DATA_LENGTH			0x81

#define MAX_DATA_LENGTH				51
#define MAX_UART_LENGTH				MAX_DATA_LENGTH + 5
#define MAX_VERSION_LENGTH				12

#define MAX_TYPE_NUM					12

#define MCU_ID_SIZE						12
#define MCU_CODER_SIZE					48


#define RSA_KEY_BIT_SIZE				128
#define TOKEN_SIZE						4
#define ENCODE_SIZE						(MCU_ID_SIZE + RSA_KEY_BIT_SIZE/8 + TOKEN_SIZE)



/*Uart send*/
#define UART_HEAD						0x2E

/*Uart send type*/
#define US_MCU_ERR						0x01					//SEND ERROR CODE

#define US_CAN_DATA						0x0A
#define US_RED_DATA						0x0B
#define US_CAN_DATA_TEST				0x0C
#define US_ADC_DATA						0x0D
#define US_GPIO_DATA					0x0E
#define US_CPUID_DATA					0x0F
#define US_MCU_PING						0x1A
#define US_MCU_CONFIG					0x1B
#define US_HOST_ID						0x1C
#define US_ID_CODER_DATA				0x1D
#define US_ID_CLEAR_DATA				0x1E

#define US_GPIO_READ					0x1F
#define US_GPIO_WRITE					0x20
#define US_GPIO_CONFIG					0x21

#define US_USART_READ					0x22
#define US_USART_WRITE					0x23
#define US_USART_CONFIG					0x24

#define US_SINGNAL_DATA					0x25
#define US_SIGNAL_CONFIG				0X26

#define US_USB_PWR_READ				0x27
#define US_USB_PWR_WRITE				0x28
#define US_USB_PWR_CONFIG				0x29

#define US_DEV_CFG_READ					0x30
#define US_DEV_CFG_WRITE				0x31

#define US_MCU_AES_READ				0x32		//AES Read
#define US_MCU_AES_WRITE				0x33		//AES Write
#define US_MCU_AES_CONFIG				0x34


#define US_MCU_ID_READ					0x35

#define US_MCU_RSA_LOCK_WRITE			0X36
#define US_MCU_RSA_LOCK_READ			0X37
#define US_MCU_RSA_LOCK_CONFIG			0X38


#define US_MCU_UPDATA_READ					0X39
#define US_MCU_UPDATA_WRITE					0X40
#define US_MCU_UPDATA_CONFIG				0X41


#define US_USART_GPS_READ						0x42
#define US_USART_GPS_WRITE					0x43
#define US_USART_GPS_CONFIG					0x44

#define US_MCU_CONF_WRITE						0x45

#define SNS_MCU_DAC_WRITE						0x46
#define SNS_MCU_DAC_READ						0x47
#define SNS_MCU_DAC_CONFIG					0x48

#define SNS_MCU_SENSOR_WRITE				0x49
#define SNS_MCU_SENSOR_READ					0x50
#define SNS_MCU_SENSOR_CONFIG				0x51

#define US_MCU_FLASH_WRITE					0X52
#define US_MCU_FLASH_READ						0X53
#define US_MCU_FLASH_CONFIG					0X54

#define US_MCU_FLASH_LOAD						0X55
#define US_MCU_FLASH_STORE					0X56
#define US_MCU_FLASH_FILL						0X57
#define US_MCU_FLASH_FETCH					0X58

#define IAP_MCU_NEED_HELP						0X59

#define SNS_iNEMO_SENSOR_WRITE			0x60

#define SJS_ROBOT_ENCODER_R					0x61
#define SJS_ROBOT_ENCODER_L					0x62
#define SJS_ROBOT_ENCODER_WRITE			0x63
#define SJS_ROBOT_ENCODER_READ			0x64
#define SJS_ROBOT_ENCODER_CONFIG		0x65
#define SJS_ROBOT_ENCODER						0x66
#define SJS_ROBOT_PWR								0x67
#define SJS_ROBOT_SPEED							0x68
#define SJS_ROBOT_WHISTLED					0x69
#define SJS_ROBOT_JOYSTICK					0x70
#define SJS_ROBOT_ULT								0x71





/*CAR INFO*/
#define US_BACK_CAR						0xC1
#define US_TIRE_CORNER_INFO				0xC2

/*ID CODER*/

#define idN								32387
#define idD								15685
#define EN_SIZE							12

#define CTRL_NUM						3
#define DEV_NUM							17

#define MAX_TRANS_INFO					40		
#define UPDATA_PKG_LENGTH				(MAX_TRANS_INFO - 12)

#define SIG_PIN1							GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)
#define SIG_PIN2 							GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)
#define SIG_PIN3 							GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)

#define ULT_ECHO_START_TIME_OUT		50000
#define ULT_ECHO_STOP_TIME_OUT			35000


#define ULT_TRIG_UP(NUM)				do{if(NUM == 0){GPIO_SetBits(GPIOA,GPIO_Pin_13);}else if(NUM == 1){GPIO_SetBits(GPIOB,GPIO_Pin_0);}}while(0)
#define ULT_TRIG_DOWN(NUM)				do{if(NUM == 0){GPIO_ResetBits(GPIOA,GPIO_Pin_13);}else if(NUM == 1){GPIO_ResetBits(GPIOB,GPIO_Pin_0);}}while(0)
#define ULT_ECHO_DATA_0					GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_14)
#define ULT_ECHO_DATA_1					GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)

#define US_DEV_NUM						40	

#define MAX_ID_SIZE						16

/*MCU ADDR***************************************************************/
#define IMAGE_VERSION_MAIN_ADDR		0x08014000
#define IMAGE_VERSION_UPDATA_ADDR		0x08014004
	
#define IMAGE_MAIN_ADDR				0x08004000

#define IMAGE_UPDATA_SIZE_ADDR			0x08014008
#define IMAGE_UPDATA_ADDR				0x0800C000


#define USART_MAX_BUFF_SEND			512

#define MAX_USART_BUFF_NUM			10
#define MAX_UASRT_BUFF_LENGRH			30


#define ApplicationAddress    				IMAGE_MAIN_ADDR

#define MCU_FLASH_DATA_ADDR			0x0800C000	
#define MCU_FLASH_DATA_CACHE		0x0800E800

#define SJS_MCU_UPDATA_ADDR			0x0800C000
#define SJS_MCU_VERSION_ADDR			0x08014000
#define SJS_MCU_DAC_ADDR				0x08014800
#define SJS_MCU_DATA_ADDR				0x08024800


#if defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)
 #define PAGE_SIZE                         (0x400)    /* 1 Kbyte */
 #define FLASH_SIZE                        (0x20000)  /* 128 KBytes */
#elif defined STM32F10X_CL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x40000)  /* 256 KBytes */
#elif defined STM32F10X_HD || defined (STM32F10X_HD_VL)
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x80000)  /* 512 KBytes */
#elif defined STM32F10X_XL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x100000) /* 1 MByte */
#else 
 #error "Please select first the STM32 device to be used (in stm32f10x.h)"    
#endif

typedef  void (*pFunction)(void);

/**************************************************************************/

typedef enum _US_DEV_CTRL{
	DEV_NULL = 0,
	DEV_ON  = 1,
	DEV_OFF = 2
}US_DEV_CTRL;


typedef enum _MCU_IO_VALUE{
	IO_DOWN = 1,
	IO_UP = 2,
	USB_PWR_ON = 3,
	USB_PWR_OFF = 4
}MCU_IO_VALUE;


/*MCU DEVICE LIST*/
typedef enum _MCU_DEVICE_LIST{	
	/*MCU GPIO*/
	/*MCU GPIO IN DEV*/
	GPIO_IN_0 = 1,
	GPIO_IN_1 = 2,
	GPIO_IN_2 = 3,
	GPIO_IN_3 = 4,
	GPIO_IN_4 = 5,
	/*MCU GPIO OUT DEV*/
	GPIO_OUT_0 = 6,
	GPIO_OUT_1 = 7,
	GPIO_OUT_2 = 8,
	GPIO_OUT_3 = 9,
	GPIO_OUT_4 = 10,
	GPIO_OUT_5 = 11,
	GPIO_OUT_6 = 12,
	GPIO_OUT_7 = 13,
	/*MCU USART*/
	USART_0 = 14,
	USART_1 = 15,	
	/*MCU USB*/
	USB_PWR_1 = 16,
	USB_PWR_2 = 17,
	USB_PWR_3 = 18,
	USB_PWR_4 = 19,
	USB_PWR_5 = 20,
	USB_PWR_6 = 21,	
	/*MCU SINGAL*/
	SIGNAL_IO_1 = 22,
	SIGNAL_IO_2 = 23,
	SIGNAL_IO_3 = 24,
	SIGNAL_IO_4 = 25,
	SIGNAL_IO_5 = 26,
	SIGNAL_IO_6 = 27,
	SIGNAL_IO_DATA = 28,
	DAC_0 = 29,
	DAC_1 = 30,
	G_SENSOR = 31,
	MCU_FLASH = 32,
	MCU_CONFIG = 33,
	MCU_DEV_CTRL = 34,
	MCU_DEV_LOCK = 35,	
	MCU_DEV_ID = 36,
	MCU_RSA_LOCK = 37,	
	MCU_UPDATA = 38
}MCU_DEVICE_LIST;



typedef struct _CAR_INFO{
	unsigned long time;
	unsigned char type;
	unsigned char data[MAX_DATA_LENGTH];
}CAR_INFO;

typedef struct _US_HOST_PING{
	unsigned char id[MAX_ID_SIZE];
	unsigned char id_length;
	unsigned char link;
	unsigned char host_status;
	unsigned char mcu_status;
}US_HOST_PING;

typedef struct _UART_INFO{
	unsigned char head;
	unsigned char version[MAX_VERSION_LENGTH];
	unsigned char type;
	unsigned char length;
	unsigned char checksum;
	unsigned char data[MAX_UART_LENGTH];
}UART_INFO;

typedef struct _US_DEV_TRANS{
	unsigned char dev;
	unsigned char status;
	unsigned char length;
	unsigned char temp;
	unsigned long cmd_id;
	unsigned char data[MAX_TRANS_INFO];
}US_DEV_TRANS;

typedef struct _SJS_MOTION_STATE{
	unsigned long long id;
	unsigned short left[4];
	unsigned short right[4];
}SJS_MOTION_STATE;

//young
typedef struct _SJS_MOVE_STATE{
	unsigned long long id;
	int x_position;
	int y_position;
	int yaw;
}SJS_MOVE_STATE;

typedef struct _SJS_ULT_STATE{
	unsigned long long id;
	unsigned int left[4];
	unsigned int right[4];
}SJS_ULT_STATE;


typedef struct _US_MCU_CONFIG_LIST{
	unsigned int BaudRate;
	unsigned int GPS_BaudRate;
}US_MCU_CONFIG_LIST;

typedef enum _MCU_DEVICE_STATUS{
	MCU_DEV_INIT = 1,
	MCU_DEV_DECODE = 2,
	MCU_DEV_LINK = 3,
	MCU_DEV_UNLINK = 4,
	MCU_DEV_START = 5,
	MCU_DEV_STOP = 6,
	MCU_DEV_ERROR = 7
}MCU_STATUS;

typedef enum _ULTRASONIC_STATUS{
	ULT_INIT = 0,
	ULT_ENABLE_UP = 1,
	ULT_ENABLE_DOWN = 2,
	ULT_CHECK_START = 3,
	ULT_CHECK_STOP = 4,
	ULT_TRANS_DATA = 5
}ULTRASONIC_STATUS;


typedef struct _US_MCU_ID_GET{
	unsigned char MCU_Version[8];
	unsigned char MCU_V[4];
	unsigned char CPUID[12];
}US_MCU_ID_GET;

typedef enum _LINK_STATUS{
	UNLINK = 0,
	LINK_OK = 1
}LINK_STATUS;


typedef enum _DAC_FUC{
	DAC_EN = 0,
	DAC_DIS = 1
}DAC_FUC;

typedef enum __SENSOR_FUC{
	SENSOR_EN = 0,
	SENSOR_DIS = 1
}SENSOR_FUC;

typedef enum __ENCODER_FUC{
	ENCODER_EN = 0,
	ENCODER_DIS = 1
}ENCODER_FUC;

typedef enum __PWR_FUC{
	ROBOT_PWR_EN	= 0,
	ROBOT_PWR_DIS = 1
}PWR_FUC;

typedef enum __SPEED_FUC{
	SPEED_UP	= 0,
	SPEED_DOWN 	= 1
}SPEED_FUC;

typedef enum __WHISTLE_FUC{
	WHISTLE_ON		= 0,
	WHISTLE_OFF 	= 1
}__WHISTLE_FUC;

typedef struct __ROBOT_JOYSTICK_TRANS{
	unsigned long long cmd;
	unsigned short x;
	unsigned short y;
	unsigned int temp;
}ROBOT_JOYSTICK_TRANS;

typedef struct _UART_RC_BUFF{
	unsigned char data[MAX_UASRT_BUFF_LENGRH];
	unsigned char length;
	unsigned char temp;
}UART_RC_BUFF;

typedef struct _US_CODER_TRANS{
	unsigned char num;
	unsigned char send_length;
	unsigned char total_length;
	unsigned char temp;
	unsigned char data[UPDATA_PKG_LENGTH];
}US_CODER_TRANS;

typedef struct _US_UPDATA_TRANS{
	unsigned int version;
	unsigned int send_length;
	unsigned int total_length;
	unsigned char data[UPDATA_PKG_LENGTH];
}US_UPDATA_TRANS;

typedef struct _US_DAC_CONFIG{
	unsigned int frequency;				//Hz
	unsigned int delay;					//s
	unsigned int times;					// 1~65530  (*0 means Infinite loop)
	unsigned int length;
	unsigned int volume;
	unsigned int addr;
}US_DAC_CONFIG;

typedef enum FLASH_UPDATA_STATUS{
	FLASH_UNLOCK = 0,
	ERASE_FLASH_PAGE,
	FLASH_DOWNLOAD,
	FLASH_LOCK,
	RUN_FLASH_TEST
}FLASH_UPDATA_STATUS;

typedef struct _US_FLASH_TRANS{
	unsigned int id;
	unsigned int address;
	unsigned int send_length;
	unsigned int total_length;
	unsigned char data[UPDATA_PKG_LENGTH];
	unsigned int v_sum;
}US_FLASH_TRANS;


typedef struct _US_DAC_PWR{
	unsigned int pwr;
	unsigned int status;
}US_DAC_PWR;


typedef struct _SENSOR_ACC_DATA{
	short acc_x;
	short acc_y;
	short acc_z;
}SENSOR_ACC_DATA;

typedef struct _SENSOR_GYRO_DATA{
	short gyro_x;
	short gyro_y;
	short gyro_z;
}SENSOR_GYRO_DATA;

typedef struct _SENSOR_MAG_DATA{
	short mag_x;
	short mag_y;
	short mag_z;
}SENSOR_MAG_DATA;

typedef struct _SENSOR_TEMP_DATA{
	short temp;
}SENSOR_TEMP_DATA;



/*Function*/
int us_init_timer5(uint16_t freq);

int us_mcu_uart_coder(UART_INFO *send);
unsigned char us_uart_checksum(UART_INFO *recv_ptr);
/*buffer circle function*/
int us_mcu_rc_buff_init(void);
int us_mcu_uart_coder(UART_INFO *send);

int us_mcu_rc_buff_enter(unsigned char type, unsigned char *send, unsigned char buff_size);
int us_mcu_rc_buff_delete(UART_INFO *str);

int us_usart_rc_buff_enter(unsigned char data);
int us_usart_rc_buff_delete(unsigned char *data);

/*MCU USB*/
int us_mcu_usb_send(unsigned char *send, unsigned int size);


/*for test*/
//int us_mcu_test_back_car(UART_INFO *send, unsigned char pwr);
int us_mcu_test_tire_corner(UART_INFO *send, char data);

void cp_mcu_delay_ms(u16 time);

void us_mcu_recave(void);

/*CPU ID*/
int us_mcu_id_get(void);

int us_mcu_send_cpu_id(void);

int us_mcu_cpuid_init(void);
int us_dev_init(void);

//young
int sjs_robot_encoder_send(unsigned char cmd, int x, int y, int yaw, unsigned long long id);
int sjs_robot_ULT_send(unsigned char cmd, unsigned int left, unsigned int right, unsigned long long id);

int robot_ultrasonic_enable(void);
/*US Func*/
int us_dev_error(unsigned char dev, unsigned char* func_name, int name_len, int status);
int sjs_robot_usb_send(unsigned char cmd, unsigned char* data, int lenght, int status);
#endif
