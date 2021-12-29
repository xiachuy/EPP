
 #include "motor.h"
 #include "math.h"
 #include "tim.h"
 #define MOTOR_1 0
 #define MOTOR_2 1
 #define MOTOR_3 2
 #define MOTOR_4 3
 int flag=0;	
extern int stop_flag1;
extern int stop_flag2;
extern int stop_flag3;
extern int all_step_1;	//总共运行步数
 extern int all_step_2;		//总共运行步数
int step_to_run[4]={5000,5000,6800,6800}; //要匀速运行的步数	//要匀速运行的步数			 总共运行步数 = ACCELERATED_SPEED_LENGTH*2 + step_to_run	 
int step_to_run_1;
int	 step_to_run_2;
int	 step_to_run_3;
int step_to_run_4;

 //函数说明：计算出每一部所需的计数值
void CalculateSModelLine(float fre_[], unsigned short period_[], float len, float fre_max, float fre_min, float flexible)
{	 all_step_1=1000;
	all_step_2=1000;
	step_to_run_1=all_step_1-100;
	step_to_run_4=all_step_2-100;
		int i=0;
		float deno ;
		float melo ;
		float delt = fre_max-fre_min;
		for(; i<len; i++)
		{
				melo = flexible* (i-len/2) / (len/2);	// x值
				deno = 1.0f / (1 + expf(-melo));	//expf is a library function of exponential(e)?	 y值
				fre_[i] = delt * deno + fre_min;
				period_[i] = (unsigned short)(4500000.0f / fre[i]); // 10000000 is the timer driver frequency
		}
		
}

	void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	 static uint32_t count[4]={0,0,0,0};
	static uint32_t num_callback[4]={0,0,0,0};
	static uint8_t status[4]={1,1,1,1};
	
	if(htim->Instance==TIM1)
	{
//		step_to_run_1=2000;
	  stop_flag1=0;		
		num_callback[0]++;
		if(num_callback[0]%2==0)
		{
			switch(status[0])
						{
								case 1://加速
											 __HAL_TIM_SET_AUTORELOAD(&htim1,period[count[0]]);
											__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,period[count[0]]/2);
											count[0]++;
											if(count[0]>=ACCELERATED_SPEED_LENGTH1)
											{
													status[0]=3;
											}												
										break;
								case 3://匀速
											 step_to_run_1--;
											 if(step_to_run_1<1)
												 status[0]=2;		 
										 break;
								case 2://减速
											 count[0]--;
											 __HAL_TIM_SET_AUTORELOAD(&htim1,period[count[0]]);
											__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,period[count[0]]/2);
											if(count[0]<1)
													status[0]=0;
										 break;
								case 0://停止
										 // 关闭通道
										TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_DISABLE);				
										__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
									HAL_TIM_Base_Stop(&htim1);
									HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
								__HAL_TIM_SET_COUNTER(&htim1,0);//计数值清零
								 status[0]=1;//因为是static 在结束完成后，下次运用必须从 status等于0开始
								
	                   break;  
								default: break;//switch 标准
						
						}
		}
}

if(htim->Instance==TIM2)
{
				stop_flag2=0;	
				num_callback[2]++;
//			 step_to_run_2=4000;
		if(num_callback[2]%2==0)
		{
							switch(status[2])
						{
								case 1://加速
											 __HAL_TIM_SET_AUTORELOAD(&htim2,period[count[2]]);
											__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,period[count[2]]/2);
											count[2]++;
											if(count[2]>=ACCELERATED_SPEED_LENGTH2)
											{
													status[2]=3;
											}												
										break;
								case 3://匀速
											 step_to_run_4--;
											 if(step_to_run_4<1)
													status[2]=2;		 
										 break;
								case 2://减速
											 count[2]--;
												__HAL_TIM_SET_AUTORELOAD(&htim2,period[count[2]]);
											__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,period[count[2]]/2);
											if(count[2]<1)
													status[2]=0;
										 break;
								case 0://停止
										 // 关闭通道
										TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_DISABLE);				
										__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
									HAL_TIM_Base_Stop(&htim2);
									HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
								__HAL_TIM_SET_COUNTER(&htim2,0);//计数值清零
								 status[2]=1;//因为是static 在结束完成后，下次运用必须从 status等于0开始
								
	                   break;  
								default: break;//switch 标准
						
						}
				}
			}
		

if(htim->Instance==TIM3)
{
				stop_flag3=0;	
				num_callback[3]++;
		if(num_callback[3]%2==0)
		{
							switch(status[3])
						{
								case 1://加速
											 __HAL_TIM_SET_AUTORELOAD(&htim3,period[count[3]]);
											__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,period[count[3]]/2);
											count[3]++;
											if(count[3]>=ACCELERATED_SPEED_LENGTH2)
											{
													status[3]=3;
											}												
										break;
								case 3://匀速
											 step_to_run_4--;
											 if(step_to_run_4<1)
												 status[3]=2;		 
										 break;
								case 2://减速
											 count[3]--;
												__HAL_TIM_SET_AUTORELOAD(&htim3,period[count[3]]);
											__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,period[count[3]]/2);
											if(count[3]<1)
													status[3]=0;
										 break;
								case 0://停止
										 // 关闭通道
										TIM_CCxChannelCmd(TIM3, TIM_CHANNEL_1, TIM_CCx_DISABLE);				
										__HAL_TIM_CLEAR_FLAG(&htim3,TIM_FLAG_CC1);
									status[3]=1;
										 break;
						
						}
				}
			}
}
//void steering_gear(TIM_HandleTypeDef htimx,int TIM_CHANNEL,int y)
//{
//	 HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL);
//	__HAL_TIM_SET_COMPARE(&htimx,TIM_CHANNEL,y);

//}
