#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "usart.h"
#include "odometry.h"

/***********************************************  ���  *****************************************************************/

float position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;
float pos_x=0,pos_y=0;

/***********************************************  ����  *****************************************************************/

extern float odometry_right,odometry_left;			//���ڵõ����������ٶ�

/**********************************************  �ж���ת��ת ***********************************************************/
extern int Flag_A_Temp, Flag_B_Temp;

/***********************************************  ����  *****************************************************************/

//float wheel_interval= 268.0859f;				  //    272.0f;        //  1.0146
//float wheel_interval=276.089f;  			  //���У��ֵ=ԭ���/0.987
float wheel_interval = 480.0f;	

float multiplier		=	4.0f;           		//��Ƶ��
float deceleration_ratio = 9.78f;  				//���ٱ�

//young_test success
//float wheel_diameter 		=	110.0f;    			//����ֱ������λmm
//float encoder_diameter 	=	700.0f;     		//������ֱ������λmm

float encoder_diameter 	=	70.0f;     			//������ֱ������λmm   С��

float pi_1_2				=	1.570796f;			 		//��/2
float pi						=	3.141593f;          //��
float pi_3_2				=	4.712389f;			 		//��*3/2
float pi_2_1				=	6.283186f;			 		//��*2
float dt						=	0.005f;             //����ʱ����5ms
float line_number		=	600.0f;      				//��������
float oriention_interval	=	0; 	 					//dtʱ���ڷ���仯ֵ

float sin_=0;        										  //�Ƕȼ���ֵ
float cos_=0;

float delta_distance=0,delta_oriention=0;   //����ʱ�������˶��ľ���

float const_frame=0,const_angle=0,distance_sum=0,distance_diff=0,one_pulse=0;

float oriention_1=0,yaw = 0;;

u8 once=1;
int right_count=0,left_count=0;

extern int ROBOT_ENCODER_EN;
/****************************************************************************************************************/

//��̼Ƽ��㺯��
void odometry(float right,float left)
{	
	if(once)  //����������һ��
	{		
		const_frame = encoder_diameter*pi/(line_number*multiplier);
		const_angle = const_frame /wheel_interval;																			//0.0003651913666667
		once=0;
	}
	
		distance_sum = 0.5f*(right+left);			//�ں̵ܶ�ʱ���ڣ�С����ʻ��·��Ϊ�����ٶȺ�
		distance_diff = right-left;						//�ں̵ܶ�ʱ���ڣ�С����ʻ�ĽǶ�Ϊ�����ٶȲ�
	
		delta_distance = distance_sum;
		delta_oriention = distance_diff;
		
		oriention_interval = delta_oriention * const_angle;		//����ʱ�����ߵĽǶ�	
		oriention = oriention + oriention_interval + (3.0/360.0) * oriention_interval;						//�������̼Ʒ����
		oriention_1 = oriention + 0.5f * oriention_interval;	//��̼Ʒ��������λ���仯���������Ǻ�������

//		printf("yaw = %f \r\n",yaw);

		sin_ = sin(oriention_1);			//���������ʱ����x����
		cos_ = cos(oriention_1);			//���������ʱ����y����

		pos_y = pos_y + delta_distance * const_frame * cos_ ;	//�������̼�y����
		pos_x = pos_x + delta_distance * const_frame * sin_ ;	//�������̼�x���� 
		
		position_x = -pos_x;
		position_y = pos_y;
		
		position_x = position_x * 1.19;
		position_y = position_y * 1.19;
			
//		printf("position_x = %f , position_y = %f \r\n",(position_x ), (position_y));
		
		velocity_linear = delta_distance*const_frame / dt;	//�������̼����ٶ�
		velocity_angular = oriention_interval / dt;					//�������̼ƽ��ٶ�

		//����ǽǶȾ���
		if(oriention > pi)
		{
			oriention -= pi_2_1;	  		//   ��*2
		}
		else
		{
			if(oriention < -pi)
			{
				oriention += pi_2_1;			//   ��*2
			}
		}
    

}
