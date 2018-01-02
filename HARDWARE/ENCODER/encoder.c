#include "encoder.h"

/****************************************************************************************************************/

s32 hSpeed_Buffer2[SPEED_BUFFER_SIZE]={0}, hSpeed_Buffer1[SPEED_BUFFER_SIZE]={0};//�������ٶȻ�������
static unsigned int hRot_Speed2;//���Aƽ��ת�ٻ���
static unsigned int hRot_Speed1;//���Bƽ��ת�ٻ���
unsigned int Speed2=0; //���Aƽ��ת�� r/min��PID����
unsigned int Speed1=0; //���Bƽ��ת�� r/min��PID����

static volatile u16 hEncoder_Timer_Overflow1;//���B�������ɼ� 
static volatile u16 hEncoder_Timer_Overflow2;//���A�������ɼ�

float A_REMP_PLUS;//���APID���ں��PWMֵ����
int span;//�ɼ��������������ٶȲ�ֵ

static bool bIs_First_Measurement2 = true;//���A������ٶȻ��������־λ
static bool bIs_First_Measurement1 = true;//���B������ٶȻ��������־λ

struct PID Control_left  ={0.01,0.1,0.75,0,0,0,0,0,0};//����PID�����������µ��4096
struct PID Control_right ={0.01,0.1,0.75,0,0,0,0,0,0};//����PID�����������µ��4096

int Flag_A_Temp = 0;	//�жϱ���������ת���Ƿ�ת -1����ת��1����ת
int Flag_B_Temp = 0;	//�жϱ���������ת���Ƿ�ת -1����ת��1����ת

/****************************************************************************************************************/

s32 hPrevious_angle2, hPrevious_angle1;

/****************************************************************************************************************/

s16 ENC_Calc_Rot_Speed2(void)//������A�ı�����
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement2)//���A������ٶȻ�������
    {  
        hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow2;
			
        hCurrent_angle_sample_one = ENCODER2_TIMER->CNT;

				hEncoder_Timer_Overflow2 = 0;
        haux = ENCODER2_TIMER->CNT;   

        if ( (ENCODER2_TIMER->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
        {
           // encoder timer down-counting ��ת���ٶȼ���     
          wDelta_angle = (s32)((hEnc_Timer_Overflow_sample_one) * (4*ENCODER2_PPR) -(hCurrent_angle_sample_one - hPrevious_angle2));
					Flag_A_Temp = -1;
					wDelta_angle = -wDelta_angle;
        }
        else  
        {
					//encoder timer up-counting ��ת���ٶȼ���
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
        bIs_First_Measurement2 = false;//���A������ٶȻ��������־λ
        temp = 0;
        hEncoder_Timer_Overflow2 = 0;
        haux = ENCODER2_TIMER->CNT;       
    }
    hPrevious_angle2 = haux;  
    return((s16) temp);
}


//NOTE:�߽ӷ�
s16 ENC_Calc_Rot_Speed1(void)//������B�ı�����
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement1)//���B������ٶȻ�������
    {   
        hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow1; 		
        hCurrent_angle_sample_one = ENCODER1_TIMER->CNT;
        hEncoder_Timer_Overflow1 = 0;
        haux = ENCODER1_TIMER->CNT;   

        if ( (ENCODER1_TIMER->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
        {
					// encoder timer down-counting ��ת���ٶȼ���
					wDelta_angle = (s32)((hEnc_Timer_Overflow_sample_one) * (4*ENCODER1_PPR) -(hCurrent_angle_sample_one - hPrevious_angle1));
					Flag_B_Temp = 1;
					
        }
        else  
        {
					//encoder timer up-counting ��ת���ٶȼ���
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
        bIs_First_Measurement1 = false;//���B������ٶȻ��������־λ
        temp = 0;
        hEncoder_Timer_Overflow1 = 0;
        haux = ENCODER1_TIMER->CNT;       
    }
    hPrevious_angle1 = haux;  
    return((s16) temp);
}


/****************************************************************************************************************/
void ENC_Clear_Speed_Buffer(void)//�ٶȴ洢������
{   
    u32 i;

    //����������ٶȻ�������
    for (i=0;i<SPEED_BUFFER_SIZE;i++)
    {
        hSpeed_Buffer2[i] = 0;
        hSpeed_Buffer1[i] = 0;
    }
    
    bIs_First_Measurement2 = true;//���A������ٶȻ��������־λ
    bIs_First_Measurement1 = true;//���B������ٶȻ��������־λ
}

void ENC_Calc_Average_Speed(void)//�������ε����ƽ��������
{   
    u32 i;
	signed long long wtemp3=0;
	signed long long wtemp4=0;

    //�ۼӻ�������ڵ��ٶ�ֵ
	for (i=0;i<SPEED_BUFFER_SIZE;i++)
	{
		wtemp4 += hSpeed_Buffer2[i];
		wtemp3 += hSpeed_Buffer1[i];
	}
    
    //ȡƽ����ƽ����������λΪ ��/s	
	wtemp3 /= (SPEED_BUFFER_SIZE);
	wtemp4 /= (SPEED_BUFFER_SIZE); //ƽ�������� ��/s	
    
    //��ƽ����������λתΪ r/min
	wtemp3 = (wtemp3 * SPEED_SAMPLING_FREQ)*60/(4*ENCODER1_PPR);
	wtemp4 = (wtemp4 * SPEED_SAMPLING_FREQ)*60/(4*ENCODER2_PPR); 
		
	hRot_Speed2= ((s16)(wtemp4));
	hRot_Speed1= ((s16)(wtemp3));
	Speed2=hRot_Speed2;//ƽ��ת�� r/min
	Speed1=hRot_Speed1;//ƽ��ת�� r/min
}

/****************************************************************************************************************/

void Gain2(void)//���õ��A PID���� PA2
{
	static float pulse = 0;
    
	span=1*(Speed1-Speed2);//�ɼ��������������ٶȲ�ֵ
	pulse= pulse + PID_calculate(&Control_right,hRot_Speed2);//PID����
    
    //��������
	if(pulse > 3600) 
        pulse = 3600;
	if(pulse < 0) 
        pulse = 0;
    
	A_REMP_PLUS=pulse;//���APID���ں��PWMֵ����
	TIM2->CCR3 = A_REMP_PLUS;//���A��ֵPWM
}


void Gain1(void)//���õ��B PID���� PA1
{
	static float pulse1 = 0;
    
	span=1*(Speed2-Speed1);//�ɼ��������������ٶȲ�ֵ
	pulse1= pulse1 + PID_calculate(&Control_left,hRot_Speed1);//PID����
    
  //��������
	if(pulse1 > 3600) 
		pulse1 = 3600;
	if(pulse1 < 0) 
		pulse1 = 0;
	
	TIM2->CCR2 = pulse1;//���B��ֵPWM
	
}

/****************************************************************************************************************/

//void ENC_Init(void)//��������ʼ��
//{
//	ENC_Init2();              //���õ��A TIM4������ģʽPB6 PB7 �ҵ��
//	ENC_Init1();              //���õ��B TIM3������ģʽPA6 PA7 ����
//	ENC_Clear_Speed_Buffer();//�ٶȴ洢������
//}

/****************************************************************************************************************/

void TIM4_IRQHandler (void)//ִ��TIM4(���A�������ɼ�)�����ж�
{   
    TIM_ClearFlag(ENCODER2_TIMER, TIM_FLAG_Update);
    if (hEncoder_Timer_Overflow2 != U16_MAX)//������Χ  
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

void TIM3_IRQHandler (void)//ִ��TIM3(���B�������ɼ�)�����ж�
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

