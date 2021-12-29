#include "robot.h"
#include "stm32f1xx_hal.h"
float L1=100;
 float x,y;//x y 分别为机械臂解算的目标值
 float r;
 float sigma1,sigma2,sigma3,sigma4,sigma5;
 float T_sigma1;
 float T_sigma2;
 float last_sigma1;
 float last_sigma2;
int all_step_1;
int all_step_2;
float L1_sigma1;
float L2_sigma2;
float last_sigma1=pi/6;
float last_sigma2;
int ii;
//解算机械臂

void robot_arm(float x,float y)
{
	x=120;
	y=80;
	//求解T_sigma0
	r=sqrt(x*x+y*y);
	sigma1=acos((r/2)/L1);
	sigma2=atan(x/y);
	sigma3=pi-sigma1-sigma2-sigma0;
	T_sigma1=sigma3;//L1臂要转过的角度
	sigma4=pi-2*sigma1;
	sigma5=sigma1+sigma2;
	
	if((r*r-2*y*L1)<0||x<0)
	{

	T_sigma2=pi-sigma1-sigma2-sigma4;//L2臂所要转过的角度
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	}
	if((r*r-2*y*L1)>0&&x>0)
	{
		T_sigma2=sigma1+sigma2+sigma4-pi;
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	
	}
}

void motor_result(float x,float y)
{
	float a1,a2;
	last_sigma1=T_sigma1;
	last_sigma2=T_sigma2;
	robot_arm(x,y);
	a1=last_sigma1*180./3.14;
	a2=last_sigma2*180./3.14;

	if(T_sigma1>last_sigma1)
	{
		L1_sigma1=T_sigma1-last_sigma1;
		all_step_1=(L1_sigma1*180/pi)/360*1600*62/20;
	}
	else if(T_sigma1<last_sigma1)
	{
		L1_sigma1=last_sigma1-T_sigma1;
		all_step_1=(L1_sigma1*180/pi)/360*1600*62/20;
	}
  if(T_sigma2>last_sigma2)
	{
			L2_sigma2=T_sigma2-last_sigma2;
			all_step_2=(L2_sigma2*180/pi)/360*1600*62/20;
	}
	else if(T_sigma2<last_sigma2)
	{
		L2_sigma2=last_sigma2-T_sigma2;
		all_step_2=(L2_sigma2*180/pi)/360*1600*62/20;
	}
}


