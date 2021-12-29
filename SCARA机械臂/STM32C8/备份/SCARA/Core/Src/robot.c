#include "robot.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "motor.h"
#include "tim.h"
#include "main.h"
float L1=100;
float L2=100;
 float x,y;//x y 分别为机械臂解算的目标值
 float r;
 float sigma3,sigma4,sigma5;
 float sigma1=-pi/6;
float sigma2=0.86111*pi;
 float T_sigma1;
 float T_sigma2;
int all_step_1;
int all_step_2;
int all_step_3;
float last_sigma1;
float last_sigma2;
float L1_sigma1;
float L2_sigma2;
float last_arm=0.136111*pi; 
float current_sigma;
//解算机械臂
// HAL_GPIO_WritePin(GPIOB, DIR4_Pin, GPIO_PIN_SET);//DIR1 Z轴低电平向无电机一侧转动
 //HAL_GPIO_WritePin(GPIOB, DIR2_Pin, GPIO_PIN_SET);//DIR2 大臂高电平逆时针转动
 //HAL_GPIO_WritePin(GPIOB, DIR3_Pin, GPIO_PIN_RESET);//DIR3 小臂高电平逆时针转动
	//CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH,FRE_MAX,FRE_MIN,4);

float actan(float x,float y)
{
	float alpha;
	if (x>0)
    	alpha=atan(y/x);
	else if(x<0&&y>=0)
	     alpha=atan(y/x)+pi;
	else if (x<0&&y<0)
	     alpha=atan(y/x)-pi;
	else if (x==0&&y>0)
	    alpha=pi/2;
	else if (x==0&&y<0)
	    alpha=-pi/2;

	return alpha;

}

void robot_arm(float x,float y)
{
	
	float c2,s2,r;
	float beta,alpha;
	float sin_b,cos_b;
	r=sqrt(x*x+y*y);
	c2=((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2));
	s2=sqrt(1-c2*c2);
	sigma2=actan(c2,s2);
	alpha=actan(x,y);
	sin_b=(L2*s2/r);
	cos_b=((L1+L2*c2)/r);
	beta=actan(cos_b,sin_b);
	sigma1=alpha-beta;
	if(sigma1<-pi)
	{
		sigma1=2*pi+sigma1;
	}
}

void motor_result(float x,float y)
{
	float a1,a2;
	last_sigma1=sigma1;
	last_sigma2=sigma2;
	a1=last_sigma1*180./3.14;
	a2=last_sigma2*180./3.14;
	robot_arm(x,y);

	if(sigma1>last_sigma1)
	{
		L1_sigma1=sigma1-last_sigma1;
		all_step_1=(fabs(L1_sigma1)*180/pi)/360*1600*48/20;
    HAL_GPIO_WritePin(GPIOB, DIR2_Pin, GPIO_PIN_SET);
	}
	else if(sigma1<last_sigma1)
	{
		L1_sigma1=sigma1-last_sigma1;
		all_step_1=(fabs(L1_sigma1)*180/pi)/360*1600*48/20;
		HAL_GPIO_WritePin(GPIOB, DIR2_Pin, GPIO_PIN_RESET);
	}

	current_sigma=L1_sigma1+last_arm;
//	if(current_sigma<pi/18)
//	{
//			current_sigma=pi/18;
//	}
	if(current_sigma>(pi-sigma2))
	{
		L2_sigma2=(pi-sigma2)-current_sigma; 
		last_arm=(pi-sigma2); 
		all_step_2=(fabs(L2_sigma2)*180/pi)/360*1600*48/20;	
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else if (current_sigma<=(pi-sigma2))
	{
		L2_sigma2=(pi-sigma2)-current_sigma; 
		last_arm=(pi-sigma2); 
		all_step_2=(fabs(L2_sigma2)*180/pi)/360*1600*48/20;	
   	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
	}
}
//void move0(float x,float y)
//{
//	if(x<=0)
//	{
//		all_step_1=(fabs(sigma1+pi/6)*180/pi)/360*1600*48/20;
//		HAL_GPIO_WritePin(GPIOB, DIR2_Pin, GPIO_PIN_RESET);
//		all_step_2=0;
//		HAL_GPIO_WritePin(GPIOC, EN3_Pin, GPIO_PIN_SET);
//	}
//	else
//	{
//		all_step_1=(fabs(sigma1+pi/6)*180/pi)/360*1600*48/20;
//		HAL_GPIO_WritePin(GPIOB, DIR2_Pin, GPIO_PIN_RESET);
//		all_step_2=(fabs(pi-sigma2-0.136111*pi-(sigma1+pi/6))*180/pi)/360*1600*48/20;
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOC, EN3_Pin, GPIO_PIN_RESET);
//	}

////	last_sigma1=-pi/6;
////	last_sigma2=0.86111*pi;
////	last_arm=0.136111*pi;
//	CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH1,FRE_MAX,FRE_MIN,4);
//	CalculateSModelLine(fre2,period2,ACCELERATED_SPEED_LENGTH2,FRE_MAX,FRE_MIN,4);
//	CalculateSModelLine(fre3,period2,ACCELERATED_SPEED_LENGTH2,FRE_MAX,FRE_MIN,4);
//	HAL_GPIO_WritePin(GPIOC, EN4_Pin|EN2_Pin|EN1_Pin, GPIO_PIN_RESET);
//// HAL_GPIO_WritePin(GPIOB, DIR4_Pin, GPIO_PIN_SET);//DIR1 Z轴低电平向无电机一侧转动
//// HAL_GPIO_WritePin(GPIOB, DIR2_Pin, GPIO_PIN_SET);//DIR2 大臂高电平逆时针转动
//// HAL_GPIO_WritePin(GPIOB, DIR3_Pin, GPIO_PIN_SET);//DIR3 小臂高电平逆时针转动
////CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH,FRE_MAX,FRE_MIN,4);
//  HAL_TIM_Base_Start(&htim1);//启动定时器
//	HAL_TIM_Base_Start(&htim2);//启动定时器
//	HAL_TIM_Base_Start(&htim3);//启动定时器
//	HAL_TIM_Base_Start(&htim4);//启动定时器
//	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);//启动定时器8通道1比较输出中断
//	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);//启动定时器8通道2比较输出中断
//	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);//启动定时器8通道1比较输出中断
//	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_3);//启动定时器8通道2比较输出中断
////  HAL_Delay(3000);
//}

