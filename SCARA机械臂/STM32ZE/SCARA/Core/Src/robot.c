#include "robot.h"
#include "stm32f1xx_hal.h"
float L1=100;
float L2=100;
 float x,y;//x y 分别为机械臂解算的目标值
 float r;
 float sigma3,sigma4,sigma5;
 float sigma1=-pi/3;
float sigma2=0.833*pi;
 float T_sigma1;
 float T_sigma2;
int all_step_1;
int all_step_2;
float last_sigma1;
float last_sigma2;
float L1_sigma1;
float L2_sigma2;
float last_arm=pi/6; 
float current_sigma;
//解算机械臂

//void robot_arm(float x,float y)
//{
//	//求解T_sigma0
//	r=sqrt(x*x+y*y);
//	sigma1=acos((r/2)/L1);
//	sigma2=atan(x/y);
//	sigma3=pi-sigma1-sigma2-sigma0;
//	T_sigma1=sigma3;//L1臂要转过的角度
//	sigma4=pi-2*sigma1;
//	sigma5=sigma1+sigma2;
//	
//	if((r*r-2*y*L1)<0||x<0)
//	{

//	T_sigma2=pi-sigma1-sigma2-sigma4;//L2臂所要转过的角度
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
//	}
//	if((r*r-2*y*L1)>0&&x>0)
//	{
//		T_sigma2=sigma1+sigma2+sigma4-pi;
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
//	
//	}
//	
//	
//}

//void motor_result(float x,float y)
//{
//	float a1,a2;
//	last_sigma1=T_sigma1;
//	last_sigma2=T_sigma2;
//	robot_arm(x,y);
//	a1=last_sigma1*180./3.14;
//	a2=last_sigma2*180./3.14;

//	if(T_sigma1>last_sigma1)
//	{
//		L1_sigma1=T_sigma1-last_sigma1;
//		all_step_1=(L1_sigma1*180/pi)/360*1600*62/20;
//	}
//	else if(T_sigma1<last_sigma1)
//	{
//		L1_sigma1=last_sigma1-T_sigma1;
//		all_step_1=(L1_sigma1*180/pi)/360*1600*62/20;
//	}
//  if(T_sigma2>last_sigma2)
//	{
//			L2_sigma2=T_sigma2-last_sigma2;
//			all_step_2=(L2_sigma2*180/pi)/360*1600*62/20;
//			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
//	}
//	else if(T_sigma2<last_sigma2)
//	{
//		L2_sigma2=last_sigma2-T_sigma2;
//		all_step_2=(L2_sigma2*180/pi)/360*1600*62/20;
//	}
//}


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
		all_step_1=(fabs(L1_sigma1)*180/pi)/360*1600*62/20;
     	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
	}
	else if(sigma1<last_sigma1)
	{
		L1_sigma1=sigma1-last_sigma1;
		all_step_1=(fabs(L1_sigma1)*180/pi)/360*1600*62/20;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
	}

	current_sigma=L1_sigma1+last_arm;
	if(current_sigma<pi/6)
	{
			current_sigma=pi/6;
	}
	if(current_sigma>(pi-sigma2))
	{
		L2_sigma2=(pi-sigma2)-current_sigma; 
		last_arm= (pi-sigma2); 
		all_step_2=(fabs(L2_sigma2)*180/pi)/360*1600*62/20;	
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	}
	else if (current_sigma<(pi-sigma2))
	{
		L2_sigma2=(pi-sigma2)-current_sigma; 
		last_arm= (pi-sigma2); 
		all_step_2=(fabs(L2_sigma2)*180/pi)/360*1600*62/20;	
   			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	}
}

void motor_reset()
{
	motor_result(20,10);
}


