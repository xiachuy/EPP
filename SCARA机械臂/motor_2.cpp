#include <stdio.h>
#include <math.h>
#define pi 3.1415
#define u   180/3.1415
int L1= 100;
int  L2= 188;
float sigma1=-pi/3;
float sigma2=pi/3;
float x,y;
float T_sigma1;
float T_sigma2;
float L1_sigma1;
float L2_sigma2;
int all_step_1;
int all_step_2;
float last_sigma1;
float last_sigma2;
float last_arm=pi/6; 
float current_sigma;

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

void scara_inverse(float x,float y)
{
	
	float c2,s2,r;
	float beta,alpha;
	float sin_b,cos_b;
	r=sqrt(x*x+y*y);
	c2=((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2));
	s2=sqrt(1-c2*c2);
	sigma2=actan(c2,s2);
	printf("sigma2为%f\n",sigma2*u);
	alpha=actan(x,y);
	sin_b=(L2*s2/r);
	cos_b=((L1+L2*c2)/r);
	beta=actan(cos_b,sin_b);
	sigma1=alpha-beta;
	printf("sigma1为%f\n",sigma1*u); 
}

void relative_position(float x,float y)
{
	float a1,a2;
	last_sigma1=sigma1;
	last_sigma2=sigma2;
	a1=last_sigma1*180./3.14;
	a2=last_sigma2*180./3.14;
	printf("上次角度1：%f\n",a1);
	printf("上次角度2：%f\n",a2);
	scara_inverse(x,y);
	printf("本次角度1：%f\n",sigma1*180/pi);
	printf("本次角度2：%f\n",sigma2*180/pi);

	if(sigma1>last_sigma1)
	{
		L1_sigma1=sigma1-last_sigma1;
		printf("电机1----------\n  ");
        printf("逆时针旋转%f\n",(L1_sigma1*180)/3.14);
		all_step_1=(abs(L1_sigma1)*180/pi)/360*1600*48/20;
        printf("电机1总步数%d\n",all_step_1);
	}
	else if(sigma1<last_sigma1)
	{
		L1_sigma1=sigma1-last_sigma1;
		all_step_1=(abs(L1_sigma1)*180/pi)/360*1600*48/20;
		printf("电机1---------- \n ");
			
        printf("顺时针旋转%f\n",((L1_sigma1)*180)/3.14);
        printf("电机1总步数%d\n",all_step_1);
	}
		current_sigma=L1_sigma1+last_arm;
	if(current_sigma<pi/18)
	{
			current_sigma=pi/18;
	}
	if(current_sigma>(pi-sigma2))
	{
		L2_sigma2=(pi-sigma2)-current_sigma; 
		last_arm=(pi-sigma2); 
		all_step_2=(fabs(L2_sigma2)*180/pi)/360*1600*48/20;	
		printf("逆时针旋转%f\n",(L2_sigma2*180)/3.14);
		printf("电机2总步数%d\n",all_step_2);
		
	}
	else if (current_sigma<=(pi-sigma2))
	{
		L2_sigma2=(pi-sigma2)-current_sigma; 
		last_arm=(pi-sigma2); 
		all_step_2=(fabs(L2_sigma2)*180/pi)/360*1600*48/20;	
		printf("顺时针旋转%f\n",(L2_sigma2*180)/3.14);
		printf("电机2总步数%d\n",all_step_2);
	}
	
}
int main()
{
	printf("输入坐标x\n");
	scanf("%f",&x);
	printf("输入坐标y\n");
	scanf("%f",&y);
	relative_position(x,y);
	printf("_______\n-");
//	relative_position(-x,y);
	printf("_______\n-");
//	relative_position(x,y-60);
	return 0;
}
