#ifndef __IIC_MPU9250_H
#define __IIC_MPU9250_H

#include "STM32F10x.h"

#define GYRO_KEN 0.06103515625
#define ACC_KEN 0.00006103515625
/* ---- MPU9250 Reg In MPU9250 ---------------------------------------------- */

#define MPU9250_I2C_ADDR            ((u8)0xD0)
#define MPU9250_Device_ID           ((u8)0x71)  // In MPU9250

#define MPU9250_SELF_TEST_XG        ((u8)0x00)
#define MPU9250_SELF_TEST_YG        ((u8)0x01)
#define MPU9250_SELF_TEST_ZG        ((u8)0x02)
#define MPU9250_SELF_TEST_XA        ((u8)0x0D)
#define MPU9250_SELF_TEST_YA        ((u8)0x0E)
#define MPU9250_SELF_TEST_ZA        ((u8)0x0F)
#define MPU9250_XG_OFFSET_H         ((u8)0x13)
#define MPU9250_XG_OFFSET_L         ((u8)0x14)
#define MPU9250_YG_OFFSET_H         ((u8)0x15)
#define MPU9250_YG_OFFSET_L         ((u8)0x16)
#define MPU9250_ZG_OFFSET_H         ((u8)0x17)
#define MPU9250_ZG_OFFSET_L         ((u8)0x18)
#define MPU9250_SMPLRT_DIV          ((u8)0x19)
#define MPU9250_CONFIG              ((u8)0x1A)
#define MPU9250_GYRO_CONFIG         ((u8)0x1B)
#define MPU9250_ACCEL_CONFIG        ((u8)0x1C)
#define MPU9250_ACCEL_CONFIG_2      ((u8)0x1D)
#define MPU9250_LP_ACCEL_ODR        ((u8)0x1E)
#define MPU9250_MOT_THR             ((u8)0x1F)
#define MPU9250_FIFO_EN             ((u8)0x23)
#define MPU9250_I2C_MST_CTRL        ((u8)0x24)
#define MPU9250_I2C_SLV0_ADDR       ((u8)0x25)
#define MPU9250_I2C_SLV0_REG        ((u8)0x26)
#define MPU9250_I2C_SLV0_CTRL       ((u8)0x27)
#define MPU9250_I2C_SLV1_ADDR       ((u8)0x28)
#define MPU9250_I2C_SLV1_REG        ((u8)0x29)
#define MPU9250_I2C_SLV1_CTRL       ((u8)0x2A)
#define MPU9250_I2C_SLV2_ADDR       ((u8)0x2B)
#define MPU9250_I2C_SLV2_REG        ((u8)0x2C)
#define MPU9250_I2C_SLV2_CTRL       ((u8)0x2D)
#define MPU9250_I2C_SLV3_ADDR       ((u8)0x2E)
#define MPU9250_I2C_SLV3_REG        ((u8)0x2F)
#define MPU9250_I2C_SLV3_CTRL       ((u8)0x30)
#define MPU9250_I2C_SLV4_ADDR       ((u8)0x31)
#define MPU9250_I2C_SLV4_REG        ((u8)0x32)
#define MPU9250_I2C_SLV4_DO         ((u8)0x33)
#define MPU9250_I2C_SLV4_CTRL       ((u8)0x34)
#define MPU9250_I2C_SLV4_DI         ((u8)0x35)
#define MPU9250_I2C_MST_STATUS      ((u8)0x36)
#define MPU9250_INT_PIN_CFG         ((u8)0x37)
#define MPU9250_INT_ENABLE          ((u8)0x38)
#define MPU9250_INT_STATUS          ((u8)0x3A)
#define MPU9250_ACCEL_XOUT_H        ((u8)0x3B)
#define MPU9250_ACCEL_XOUT_L        ((u8)0x3C)
#define MPU9250_ACCEL_YOUT_H        ((u8)0x3D)
#define MPU9250_ACCEL_YOUT_L        ((u8)0x3E)
#define MPU9250_ACCEL_ZOUT_H        ((u8)0x3F)
#define MPU9250_ACCEL_ZOUT_L        ((u8)0x40)
#define MPU9250_TEMP_OUT_H          ((u8)0x41)
#define MPU9250_TEMP_OUT_L          ((u8)0x42)
#define MPU9250_GYRO_XOUT_H         ((u8)0x43)
#define MPU9250_GYRO_XOUT_L         ((u8)0x44)
#define MPU9250_GYRO_YOUT_H         ((u8)0x45)
#define MPU9250_GYRO_YOUT_L         ((u8)0x46)
#define MPU9250_GYRO_ZOUT_H         ((u8)0x47)
#define MPU9250_GYRO_ZOUT_L         ((u8)0x48)
#define MPU9250_EXT_SENS_DATA_00    ((u8)0x49)
#define MPU9250_EXT_SENS_DATA_01    ((u8)0x4A)
#define MPU9250_EXT_SENS_DATA_02    ((u8)0x4B)
#define MPU9250_EXT_SENS_DATA_03    ((u8)0x4C)
#define MPU9250_EXT_SENS_DATA_04    ((u8)0x4D)
#define MPU9250_EXT_SENS_DATA_05    ((u8)0x4E)
#define MPU9250_EXT_SENS_DATA_06    ((u8)0x4F)
#define MPU9250_EXT_SENS_DATA_07    ((u8)0x50)
#define MPU9250_EXT_SENS_DATA_08    ((u8)0x51)
#define MPU9250_EXT_SENS_DATA_09    ((u8)0x52)
#define MPU9250_EXT_SENS_DATA_10    ((u8)0x53)
#define MPU9250_EXT_SENS_DATA_11    ((u8)0x54)
#define MPU9250_EXT_SENS_DATA_12    ((u8)0x55)
#define MPU9250_EXT_SENS_DATA_13    ((u8)0x56)
#define MPU9250_EXT_SENS_DATA_14    ((u8)0x57)
#define MPU9250_EXT_SENS_DATA_15    ((u8)0x58)
#define MPU9250_EXT_SENS_DATA_16    ((u8)0x59)
#define MPU9250_EXT_SENS_DATA_17    ((u8)0x5A)
#define MPU9250_EXT_SENS_DATA_18    ((u8)0x5B)
#define MPU9250_EXT_SENS_DATA_19    ((u8)0x5C)
#define MPU9250_EXT_SENS_DATA_20    ((u8)0x5D)
#define MPU9250_EXT_SENS_DATA_21    ((u8)0x5E)
#define MPU9250_EXT_SENS_DATA_22    ((u8)0x5F)
#define MPU9250_EXT_SENS_DATA_23    ((u8)0x60)
#define MPU9250_I2C_SLV0_DO         ((u8)0x63)
#define MPU9250_I2C_SLV1_DO         ((u8)0x64)
#define MPU9250_I2C_SLV2_DO         ((u8)0x65)
#define MPU9250_I2C_SLV3_DO         ((u8)0x66)
#define MPU9250_I2C_MST_DELAY_CTRL  ((u8)0x67)
#define MPU9250_SIGNAL_PATH_RESET   ((u8)0x68)
#define MPU9250_MOT_DETECT_CTRL     ((u8)0x69)
#define MPU9250_USER_CTRL           ((u8)0x6A)
#define MPU9250_PWR_MGMT_1          ((u8)0x6B)
#define MPU9250_PWR_MGMT_2          ((u8)0x6C)
#define MPU9250_FIFO_COUNTH         ((u8)0x72)
#define MPU9250_FIFO_COUNTL         ((u8)0x73)
#define MPU9250_FIFO_R_W            ((u8)0x74)
#define MPU9250_WHO_AM_I            ((u8)0x75)	// ID = 0x71 In MPU9250
#define MPU9250_XA_OFFSET_H         ((u8)0x77)
#define MPU9250_XA_OFFSET_L         ((u8)0x78)
#define MPU9250_YA_OFFSET_H         ((u8)0x7A)
#define MPU9250_YA_OFFSET_L         ((u8)0x7B)
#define MPU9250_ZA_OFFSET_H         ((u8)0x7D)
#define MPU9250_ZA_OFFSET_L         ((u8)0x7E)

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */

#define AK8963_I2C_ADDR             ((u8)0x18)
#define AK8963_Device_ID            ((u8)0x48)

// Read-only Reg
#define AK8963_WIA                  ((u8)0x00)
#define AK8963_INFO                 ((u8)0x01)
#define AK8963_ST1                  ((u8)0x02)
#define AK8963_HXL                  ((u8)0x03)
#define AK8963_HXH                  ((u8)0x04)
#define AK8963_HYL                  ((u8)0x05)
#define AK8963_HYH                  ((u8)0x06)
#define AK8963_HZL                  ((u8)0x07)
#define AK8963_HZH                  ((u8)0x08)
#define AK8963_ST2                  ((u8)0x09)
// Write/Read Reg
#define AK8963_CNTL1                ((u8)0x0A)
#define AK8963_CNTL2                ((u8)0x0B)
#define AK8963_ASTC                 ((u8)0x0C)
#define AK8963_TS1                  ((u8)0x0D)
#define AK8963_TS2                  ((u8)0x0E)
#define AK8963_I2CDIS               ((u8)0x0F)
// Read-only Reg ( ROM )
#define AK8963_ASAX                 ((u8)0x10)
#define AK8963_ASAY                 ((u8)0x11)
#define AK8963_ASAZ                 ((u8)0x12)
typedef struct _SJSMPU
{
	s16 acc_x;
	s16 acc_y;
	s16 acc_z;
	s16 temp;
	s16 gyro_x;
	s16 gyro_y;
	s16 gyro_z;
	s16 mag_x;
	s16 mag_y;
	s16 mag_z;
}SJSMPU;
extern SJSMPU mpu9250;
void IIC_Write_OneByte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u8 IIC_Read_OneByte(u8 SlaveAddress,u8 REG_Address);
void IIC_MPU9250_init(void);
void GetMPU9250Data(void);
int get_gyro_bias(void);
#endif

