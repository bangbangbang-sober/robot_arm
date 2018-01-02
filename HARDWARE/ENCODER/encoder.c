#include "encoder.h"

/****************************************************************************************************************/

s32 hSpeed_Buffer2[SPEED_BUFFER_SIZE]={0}, hSpeed_Buffer1[SPEED_BUFFER_SIZE]={0};//左右轮速度缓存数组
static unsigned int hRot_Speed2;//电机A平均转速缓存
static unsigned int hRot_Speed1;//电机B平均转速缓存
unsigned int Speed2=0; //电机A平均转速 r/min，PID调节
unsigned int Speed1=0; //电机B平均转速 r/min，PID调节

static volatile u16 hEncoder_Timer_Overflow1;//电机B编码数采集 
static volatile u16 hEncoder_Timer_Overflow2;//电机A编码数采集

float A_REMP_PLUS;//电机APID调节后的PWM值缓存
int span;//采集回来的左右轮速度差值

static bool bIs_First_Measurement2 = true;//电机A以清除速度缓存数组标志位
static bool bIs_First_Measurement1 = true;//电机B以清除速度缓存数组标志位

struct PID Control_left  ={0.01,0.1,0.75,0,0,0,0,0,0};//左轮PID参数，适于新电机4096
struct PID Control_right ={0.01,0.1,0.75,0,0,0,0,0,0};//右轮PID参数，适于新电机4096

int Flag_A_Temp = 0;	//判断编码器是正转还是反转 -1：反转；1：正转
int Flag_B_Temp = 0;	//判断编码器是正转还是反转 -1：反转；1：正转

/****************************************************************************************************************/

s32 hPrevious_angle2, hPrevious_angle1;

/****************************************************************************************************************/

s16 ENC_Calc_Rot_Speed2(void)//计算电机A的编码数
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement2)//电机A以清除速度缓存数组
    {  
        hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow2;
			
        hCurrent_angle_sample_one = ENCODER2_TIMER->CNT;

				hEncoder_Timer_Overflow2 = 0;
        haux = ENCODER2_TIMER->CNT;   

        if ( (ENCODER2_TIMER->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
        {
           // encoder timer down-counting 反转的速度计算     
          wDelta_angle = (s32)((hEnc_Timer_Overflow_sample_one) * (4*ENCODER2_PPR) -(hCurrent_angle_sample_one - hPrevious_angle2));
					Flag_A_Temp = -1;
					wDelta_angle = -wDelta_angle;
        }
        else  
        {
					//encoder timer up-counting 正转的速度计算
					wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle2 + (hEnc_Timer_Overflow_sample_one) * (4*ENCODER2_PPR));
					Flag_A_Temp = 1;
        }
				
				if(0 == wDelta_angle){
					Flag_A_Temp = 0;
				}
        temp=wDelta_angle;
    } 
    else
    {
        bIs_First_Measurement2 = false;//电机A以清除速度缓存数组标志位
        temp = 0;
        hEncoder_Timer_Overflow2 = 0;
        haux = ENCODER2_TIMER->CNT;       
    }
    hPrevious_angle2 = haux;  
    return((s16) temp);
}


//NOTE:线接反
s16 ENC_Calc_Rot_Speed1(void)//计算电机B的编码数
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement1)//电机B以清除速度缓存数组
    {   
        hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow1; 		
        hCurrent_angle_sample_one = ENCODER1_TIMER->CNT;
        hEncoder_Timer_Overflow1 = 0;
        haux = ENCODER1_TIMER->CNT;   

        if ( (ENCODER1_TIMER->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
        {
					// encoder timer down-counting 反转的速度计算
					wDelta_angle = (s32)((hEnc_Timer_Overflow_sample_one) * (4*ENCODER1_PPR) -(hCurrent_angle_sample_one - hPrevious_angle1));
					Flag_B_Temp = 1;
					
        }
        else  
        {
					//encoder timer up-counting 正转的速度计算
					wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle1 + (hEnc_Timer_Overflow_sample_one) * (4*ENCODER1_PPR));			
	
					Flag_B_Temp = -1;
					wDelta_angle = -wDelta_angle;
				}
				
				if(0 == wDelta_angle){
					Flag_B_Temp = 0;
				}
        temp=wDelta_angle;
		
    } 
    else
    {
        bIs_First_Measurement1 = false;//电机B以清除速度缓存数组标志位
        temp = 0;
        hEncoder_Timer_Overflow1 = 0;
        haux = ENCODER1_TIMER->CNT;       
    }
    hPrevious_angle1 = haux;  
    return((s16) temp);
}


/****************************************************************************************************************/
void ENC_Clear_Speed_Buffer(void)//速度存储器清零
{   
    u32 i;

    //清除左右轮速度缓存数组
    for (i=0;i<SPEED_BUFFER_SIZE;i++)
    {
        hSpeed_Buffer2[i] = 0;
        hSpeed_Buffer1[i] = 0;
    }
    
    bIs_First_Measurement2 = true;//电机A以清除速度缓存数组标志位
    bIs_First_Measurement1 = true;//电机B以清除速度缓存数组标志位
}

void ENC_Calc_Average_Speed(void)//计算三次电机的平均编码数
{   
    u32 i;
	signed long long wtemp3=0;
	signed long long wtemp4=0;

    //累加缓存次数内的速度值
	for (i=0;i<SPEED_BUFFER_SIZE;i++)
	{
		wtemp4 += hSpeed_Buffer2[i];
		wtemp3 += hSpeed_Buffer1[i];
	}
    
    //取平均，平均脉冲数单位为 个/s	
	wtemp3 /= (SPEED_BUFFER_SIZE);
	wtemp4 /= (SPEED_BUFFER_SIZE); //平均脉冲数 个/s	
    
    //将平均脉冲数单位转为 r/min
	wtemp3 = (wtemp3 * SPEED_SAMPLING_FREQ)*60/(4*ENCODER1_PPR);
	wtemp4 = (wtemp4 * SPEED_SAMPLING_FREQ)*60/(4*ENCODER2_PPR); 
		
	hRot_Speed2= ((s16)(wtemp4));
	hRot_Speed1= ((s16)(wtemp3));
	Speed2=hRot_Speed2;//平均转速 r/min
	Speed1=hRot_Speed1;//平均转速 r/min
}

/****************************************************************************************************************/

void Gain2(void)//设置电机A PID调节 PA2
{
	static float pulse = 0;
    
	span=1*(Speed1-Speed2);//采集回来的左右轮速度差值
	pulse= pulse + PID_calculate(&Control_right,hRot_Speed2);//PID调节
    
    //幅度抑制
	if(pulse > 3600) 
        pulse = 3600;
	if(pulse < 0) 
        pulse = 0;
    
	A_REMP_PLUS=pulse;//电机APID调节后的PWM值缓存
	TIM2->CCR3 = A_REMP_PLUS;//电机A赋值PWM
}


void Gain1(void)//设置电机B PID调节 PA1
{
	static float pulse1 = 0;
    
	span=1*(Speed2-Speed1);//采集回来的左右轮速度差值
	pulse1= pulse1 + PID_calculate(&Control_left,hRot_Speed1);//PID调节
    
  //幅度抑制
	if(pulse1 > 3600) 
		pulse1 = 3600;
	if(pulse1 < 0) 
		pulse1 = 0;
	
	TIM2->CCR2 = pulse1;//电机B赋值PWM
	
}

/****************************************************************************************************************/

//void ENC_Init(void)//电机处理初始化
//{
//	ENC_Init2();              //设置电机A TIM4编码器模式PB6 PB7 右电机
//	ENC_Init1();              //设置电机B TIM3编码器模式PA6 PA7 左电机
//	ENC_Clear_Speed_Buffer();//速度存储器清零
//}

/****************************************************************************************************************/

void TIM4_IRQHandler (void)//执行TIM4(电机A编码器采集)计数中断
{   
    TIM_ClearFlag(ENCODER2_TIMER, TIM_FLAG_Update);
    if (hEncoder_Timer_Overflow2 != U16_MAX)//不超范围  
    {
        hEncoder_Timer_Overflow2++; 
    }
}

extern unsigned int A_Step_Set, A_Step_Num;
extern unsigned int B_Step_Set, B_Step_Num;
extern unsigned int C_Step_Set, C_Step_Num;

extern u16 A_FREQ, B_FREQ, C_FREQ;
extern u16 START_FREQ;

u16 capture = 0;
u16 Temp_A = 0, Temp_B = 0, Temp_C = 0;

void TIM3_IRQHandler (void)//执行TIM3(电机B编码器采集)计数中断
{  
   if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		
		if(A_Step_Set != 0){
			u16 Step_now = START_FREQ+Temp_A*(MARM_VN-2);
			
			if((Step_now < A_FREQ) && (A_Step_Num < A_Step_Set/2)){
				capture = TIM_GetCapture1(TIM3);
				TIM_SetCompare1(TIM3, capture + MY_FREQ/Step_now);
				Temp_A = A_Step_Num;
				
			}else if(A_Step_Num >= A_Step_Set - Temp_A){
				capture = TIM_GetCapture1(TIM3);
				TIM_SetCompare1(TIM3, capture + MY_FREQ/(Step_now - (A_Step_Num - (A_Step_Set - Temp_A))*(MARM_VN-2)));
			}else{
				capture = TIM_GetCapture1(TIM3);
				TIM_SetCompare1(TIM3, capture + MY_FREQ/Step_now);
			}

			if(A_Step_Num > A_Step_Set){
				A_Step_Set = 0;
				A_Step_Num = 0;
				Temp_A = 0;
				sjs_marm_pwr(SJS_M_ARM_A, ROBOT_PWR_DIS);
				sjs_marm_usb_send(SJS_MARM_CTRL, SJS_M_ARM_A, "MARM A", 7, A_Step_Set);
			}
			
			A_Step_Num++;
		}

	}

	if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		
		if(B_Step_Set != 0){
			u16 Step_now = START_FREQ+Temp_B*MARM_VN;
			
			if((Step_now < B_FREQ) && (B_Step_Num < B_Step_Set/2)){
				capture = TIM_GetCapture2(TIM3);
				TIM_SetCompare2(TIM3, capture + MY_FREQ/Step_now);
				Temp_B = B_Step_Num;
				
			}else if(B_Step_Num >= B_Step_Set - Temp_B){
				capture = TIM_GetCapture2(TIM3);
				TIM_SetCompare2(TIM3, capture + MY_FREQ/(Step_now - (B_Step_Num - (B_Step_Set - Temp_B))*MARM_VN));
			}else{
				capture = TIM_GetCapture2(TIM3);
				TIM_SetCompare2(TIM3, capture + MY_FREQ/Step_now);
			}

			if(B_Step_Num > B_Step_Set){
				B_Step_Set = 0;
				B_Step_Num = 0;
				Temp_B = 0;
				sjs_marm_pwr(SJS_M_ARM_B, ROBOT_PWR_DIS);
				sjs_marm_usb_send(SJS_MARM_CTRL, SJS_M_ARM_B, "MARM B", 7, B_Step_Set);
			}
			
			B_Step_Num++;
		}

	}

	
	if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		
		if(C_Step_Set != 0){
			u16 Step_now = START_FREQ+Temp_C*MARM_VN;
			
			if((Step_now < C_FREQ) && (C_Step_Num < C_Step_Set/2)){
				capture = TIM_GetCapture3(TIM3);
				TIM_SetCompare3(TIM3, capture + MY_FREQ/Step_now);
				Temp_C = C_Step_Num;
				
			}else if(C_Step_Num >= C_Step_Set - Temp_C){
				capture = TIM_GetCapture3(TIM3);
				TIM_SetCompare3(TIM3, capture + MY_FREQ/(Step_now - (C_Step_Num - (C_Step_Set - Temp_C))*MARM_VN));
			}else{
				capture = TIM_GetCapture3(TIM3);
				TIM_SetCompare3(TIM3, capture + MY_FREQ/Step_now);
			}

			if(C_Step_Num > C_Step_Set){
				C_Step_Set = 0;
				C_Step_Num = 0;
				Temp_C = 0;
				sjs_marm_pwr(SJS_M_ARM_C, ROBOT_PWR_DIS);
				sjs_marm_usb_send(SJS_MARM_CTRL, SJS_M_ARM_C, "MARM C", 7, C_Step_Set);
			}
			
			C_Step_Num++;
		}

	}
}

