#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "User_Control.h"

//根据编码器的线数 600（脉冲数600*4）,  主轴直径  38mm   得到一个脉冲对应长度
#define Get_OnePulse_length  (3.1415926 * 38 / (600 * 4))

#define PWMPeriod   2400
#define prd    			2400
#define Vbreak 			2200

u16 Cnt_Left;
s32 CNT_L;
s32 V_Left;

extern s32 Rcnt_L;

void get_encoder_Left(void)//*******************计算当前实际速度
{
  s32 CNTL_temp,CNTL_last;
  
  Cnt_Left = TIM3 -> CNT;
  CNTL_last = CNT_L;
  CNTL_temp = Rcnt_L * prd + Cnt_Left;  
  V_Left = CNTL_temp - CNTL_last;		
  
  while (V_Left>Vbreak)				 
  {							      
    Rcnt_L--;					      
    CNTL_temp = Rcnt_L * prd + Cnt_Left;
    V_Left = CNTL_temp - CNTL_last;		 
  }							     
  while (V_Left<-Vbreak)			   
  {							      
    Rcnt_L++;					      
    CNTL_temp = Rcnt_L * prd + Cnt_Left;
    V_Left = CNTL_temp - CNTL_last;		 
  }
	
  CNT_L = CNTL_temp;						 
  
}

void UserMotorSpeedSetOne(s32 control)//电机转速控制
{		
	s32 MotorSpeed;

	MotorSpeed = control;//读取PID的输入值
											   
	if(MotorSpeed > PWMPeriod)  
		MotorSpeed =   PWMPeriod-1 ;
	
	TIM2->CCR2 = MotorSpeed; 
}


//绝对式PID算法
void PID_AbsoluteMode(PID_AbsoluteType* PID)
{
    if(PID->kp < 0)    
        PID->kp = -PID->kp;
    if(PID->ki < 0)    
        PID->ki = -PID->ki;
    if(PID->kd < 0)    
        PID->kd = -PID->kd;
    if(PID->errILim < 0)    
        PID->errILim = -PID->errILim;

    PID->errP = PID->errNow;  //读取现在的误差，用于KP控制

    PID->errI += PID->errNow; //误差积分，用于KI控制

    if(PID->errILim != 0)	   //微分上限和下限
    {
        if(PID->errI >  PID->errILim)    
            PID->errI =  PID->errILim;
        else if(PID->errI < -PID->errILim)    
            PID->errI = -PID->errILim;
    }

    PID->errD = PID->errNow - PID->errOld;//误差微分，用于kd控制

    PID->errOld = PID->errNow;	//保存现有的误差

    PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//计算绝对式PID输出

}

s32 spdTag, spdNow, control;	//定义一个目标速度，采样速度，控制量
PID_AbsoluteType PID_Control;	//PID算法结构体

//pid控制转速
void User_PidSpeedControl(s32 SpeedTag)
{
   spdNow = V_Left; spdTag = SpeedTag;

   PID_Control.errNow = spdTag - spdNow; //计算并写入速度误差
   	
   PID_Control.kp      = 15;             //比例系数：15
   PID_Control.ki      = 5;              //积分系数：5
   PID_Control.kd      = 5;              //微分系数：5
   PID_Control.errILim = 1000;           //误差积分上限：1000 下限：-1000

   PID_AbsoluteMode(&PID_Control);       //执行绝对式PID算法
	
   control = PID_Control.ctrOut;         //读取控制值

   UserMotorSpeedSetOne(control);        //放入PWM，用于收敛速度控制中

}

//得到运行长度，编码器需要输出多少个脉冲数
int Get_Pulse(u32 length)
{
	return (length / Get_OnePulse_length);
}










