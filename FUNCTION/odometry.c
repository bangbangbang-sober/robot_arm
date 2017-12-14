#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "usart.h"
#include "odometry.h"

/***********************************************  输出  *****************************************************************/

float position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;
float pos_x=0,pos_y=0;

/***********************************************  输入  *****************************************************************/

extern float odometry_right,odometry_left;			//串口得到的左右轮速度

/**********************************************  判断正转反转 ***********************************************************/
extern int Flag_A_Temp, Flag_B_Temp;

/***********************************************  变量  *****************************************************************/

//float wheel_interval= 268.0859f;				  //    272.0f;        //  1.0146
//float wheel_interval=276.089f;  			  //轴距校正值=原轴距/0.987
float wheel_interval = 250.0f;	

float multiplier		=	4.0f;           		//倍频数
float deceleration_ratio = 9.78f;  				//减速比

//young_test success
//float wheel_diameter 		=	110.0f;    			//轮子直径，单位mm
float encoder_diameter 	=	700.0f;     		//编码器直径，单位mm

//float encoder_diameter 	=	70.0f;     		//编码器直径，单位mm

float pi_1_2				=	1.570796f;			 		//π/2
float pi						=	3.141593f;          //π
float pi_3_2				=	4.712389f;			 		//π*3/2
float pi_2_1				=	6.283186f;			 		//π*2
float dt						=	0.005f;             //采样时间间隔5ms
float line_number		=	600.0f;      				//码盘线数
float oriention_interval	=	0; 	 					//dt时间内方向变化值

float sin_=0;        										  //角度计算值
float cos_=0;

float delta_distance=0,delta_oriention=0;   //采样时间间隔内运动的距离

float const_frame=0,const_angle=0,distance_sum=0,distance_diff=0,one_pulse=0;

float oriention_1=0;

u8 once=1;
int right_count=0,left_count=0;

extern int ROBOT_ENCODER_EN;
/****************************************************************************************************************/

//里程计计算函数
void odometry(float right,float left)
{	
	if(once)  //常数仅计算一次
	{		
//		//test_success
		const_frame = encoder_diameter*pi/(line_number*multiplier); 								//0.9162978416666667
		const_angle = const_frame/wheel_interval;	
		
//		const_frame = encoder_diameter*pi/(line_number*multiplier); 								  //0.0916297841666667 
//		const_angle = const_frame/wheel_interval;																			//0.0003651913666667
		once=0;
	}
	
//young_test
//		right_count = right_count + right;
//		left_count = left_count + left;
//		printf("right_count = %d , left_count = %d \r\n",(right_count * 2) , (left_count * 2));
	
//test_success		
		distance_sum = 0.5f*(right+left);			//在很短的时间内，小车行驶的路程为两轮速度和
		distance_diff = right-left;						//在很短的时间内，小车行驶的角度为两轮速度差
	
		delta_distance = distance_sum;
		delta_oriention = distance_diff;
	
/*	
#if 0
		//根据编码反馈回来的正转还是反转标志位，来确定计算公式
		odometry_right = Flag_A_Temp; //Flag_A_Temp
		odometry_left  = Flag_B_Temp; //Flag_B_Temp
	
		//根据左右轮的方向，纠正短时间内，小车行驶的路程和角度量的正负
		if((odometry_right >= 0)&&(odometry_left >= 0))            //左右均正
		{
			delta_distance = distance_sum;
			delta_oriention = distance_diff;
		}
		else if((odometry_right < 0)&&(odometry_left < 0))       //左右均负
		{
			delta_distance = -distance_sum;
			delta_oriention = -distance_diff;
		}
		else if((odometry_right < 0)&&(odometry_left > 0))       //左正右负
		{
			delta_distance = -distance_diff;
			delta_oriention = -2.0f*distance_sum;		
		}
		else if((odometry_right > 0)&&(odometry_left < 0))       //左负右正
		{
			delta_distance = distance_diff;
			delta_oriention = 2.0f*distance_sum;
		}
		else
		{
			delta_distance=0;
			delta_oriention=0;
		}
#endif
*/
		
		oriention_interval = delta_oriention * const_angle;		//采样时间内走的角度	
		oriention = oriention + oriention_interval;						//计算出里程计方向角
		oriention_1 = oriention + 0.5f * oriention_interval;	//里程计方向角数据位数变化，用于三角函数计算

//		printf("oriention = %f \r\n",(oriention * (180/pi)));

		sin_ = sin(oriention_1);			//计算出采样时间内x坐标
		cos_ = cos(oriention_1);			//计算出采样时间内y坐标

		pos_y = pos_y + delta_distance * const_frame * cos_ ;	//计算出里程计y坐标
		pos_x = pos_x + delta_distance * const_frame * sin_ ;	//计算出里程计x坐标 
		
		position_x = -pos_x;
		position_y = pos_y;
		
		position_x = position_x * 2;
		position_y = position_y * 2;
		
//		printf("position_x = %f , position_y = %f \r\n",(position_x), (position_y));
		
		velocity_linear = delta_distance*const_frame / dt;	//计算出里程计线速度
		velocity_angular = oriention_interval / dt;					//计算出里程计角速度

		//方向角角度纠正
		if(oriention > pi)
		{
			oriention -= pi_2_1;	  		//   π*2
		}
		else
		{
			if(oriention < -pi)
			{
				oriention += pi_2_1;			//   π*2
			}
		}
    

}
