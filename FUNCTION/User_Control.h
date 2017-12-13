#ifndef __FUNTION_H
#define __FUNTION_H


#include "stm32f10x.h"
#include "stm32f10x_it.h"


/*PID 算法，接口参式峁过类型*/
typedef struct 
{

 float kp;     //比例系数
 float ki;     //积分系数
 float kd;     //微分系数
 float errILim;//误差积分上限
 
 float errNow;//当前的误差
 float ctrOut;//控制量输出
 
 
 float errOld;
 float errP;
 float errI;
 float errD;
 
}PID_AbsoluteType;

void get_encoder_Left(void);
extern void User_PidSpeedControl(s32 SpeedTag);


#endif





