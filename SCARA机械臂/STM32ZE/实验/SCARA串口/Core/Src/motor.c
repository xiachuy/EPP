 
 #include "motor.h"
 #include "math.h"
 #include "tim.h"
 
int flag=0;	
int stop_flag=0;
extern int all_step_1;    //总共运行步数
extern int all_step_2;    //总共运行步数
int step_to_run[4]={5000,5000,6800,6800}; //要匀速运行的步数  //要匀速运行的步数       总共运行步数 = ACCELERATED_SPEED_LENGTH*2 + step_to_run	 
int step_to_run_1;
int	step_to_run_2;
int stop_flag2;
int stop_flag1;
 //函数说明：计算出每一部所需的计数值

void CalculateSModelLine(float fre_[], unsigned short period_[], float len, float fre_max, float fre_min, float flexible)
{   
	  step_to_run_1=all_step_1-100;
	 step_to_run_2=all_step_2-100;
    int i=0;
    float deno ;
    float melo ;
    float delt = fre_max-fre_min;
    for(; i<len; i++)
    {
        melo = flexible* (i-len/2) / (len/2);  // x值
        deno = 1.0f / (1 + expf(-melo));  //expf is a library function of exponential(e)?   y值
        fre_[i] = delt * deno + fre_min;
        period_[i] = (unsigned short)(4500000.0f / fre[i]); // 10000000 is the timer driver frequency
    }
    
}


//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	 static uint32_t count[4]={0,0,0,0};
//	static uint32_t num_callback[4]={0,0,0,0};
//	static uint8_t status[4]={1,1,1,1};
//	
//	if(htim->Instance==TIM8)
//	{
////		step_to_run_1=2000;
//	  stop_flag1=0;		
//		num_callback[0]++;
//		if(num_callback[0]%2==0)
//		{
//			switch(status[0])
//						{
//								case 1://加速
//											 __HAL_TIM_SET_AUTORELOAD(&htim8,period[count[0]]);
//											__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,period[count[0]]/2);
//											count[0]++;
//											if(count[0]>=ACCELERATED_SPEED_LENGTH1)
//											{
//													status[0]=3;
//											}												
//										break;
//								case 3://匀速
//											 step_to_run_1--;
//											 if(step_to_run_1<1)
//												 status[0]=2;		 
//										 break;
//								case 2://减速
//											 count[0]--;
//											 __HAL_TIM_SET_AUTORELOAD(&htim8,period[count[0]]);
//											__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,period[count[0]]/2);
//											if(count[0]<1)
//													status[0]=0;
//										 break;
//								case 0://停止
//										 // 关闭通道
//										TIM_CCxChannelCmd(TIM8, TIM_CHANNEL_1, TIM_CCx_DISABLE);				
//										__HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_CC1);
//									//HAL_TIM_Base_Stop(&htim8);
//									HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_1);
//								__HAL_TIM_SET_COUNTER(&htim8,0);//计数值清零
//								 status[0]=1;//因为是static 在结束完成后，下次运用必须从 status等于0开始
//								
//	                   break;  
//								default: break;//switch 标准
//										 
//						
//						}
//		}

//	
//}

//if(htim->Instance==TIM5)
//{
//				num_callback[2]++;
//		if(num_callback[2]%2==0)
//		{
//							switch(status[2])
//						{
//								case 1://加速
//											 __HAL_TIM_SET_AUTORELOAD(&htim5,period[count[2]]);
//											__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,period[count[2]]/2);
//											count[2]++;
//											if(count[2]>=ACCELERATED_SPEED_LENGTH2)
//											{
//													status[2]=3;
//											}												
//										break;
//								case 3://匀速
//											 step_to_run_2--;
//											 if(step_to_run_2<1)
//												 status[2]=2;		 
//										 break;
//								case 2://减速
//											 count[2]--;
//												__HAL_TIM_SET_AUTORELOAD(&htim5,period[count[2]]);
//											__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,period[count[2]]/2);
//											if(count[2]<1)
//													status[2]=0;
//										 break;
//								case 0://停止
//										 // 关闭通道
//										TIM_CCxChannelCmd(TIM5, TIM_CHANNEL_2, TIM_CCx_DISABLE);				
//										__HAL_TIM_CLEAR_FLAG(&htim5,TIM_FLAG_CC2);
//								
//										HAL_TIM_Base_Stop(&htim5);
//										HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_2);
//										__HAL_TIM_SET_COUNTER(&htim5,0);//计数值清零
//									 status[1]=1;//因为是static 在结束完成后，下次运用必须从 status等于0开始
//								
//	                   break;  
//										default: break;//switch 标准
//									
//						
//						}
//				}
//		// __HAL_TIM_CLEAR_FLAG(&htim5,TIM_FLAG_CC2);
//		
//		
//			}
//	stop_flag1=1;
//}



////以前的代码
//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	static uint32_t count[4]={0,0,0,0};
//	static uint32_t num_callback[4]={0,0,0,0};
//	static uint8_t status[4]={1,1,1,1};
//		stop_flag1=0;
//	if(htim->Instance==TIM8)
////	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//	{
//        
//		num_callback[0]++;
//		if(num_callback[0]%2==0)
//		{
//			switch(status[0])
//	          {
//	              case 1://加速
//                       __HAL_TIM_SET_AUTORELOAD(&htim8,period[count[0]]);
//	                    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,period[count[0]]/2);
//	                    count[0]++;
//	                    if(count[0]>=ACCELERATED_SPEED_LENGTH1)
//	                    {
//	                        status[0]=3;
//	                    }                        
//	                  break;
//	              case 3://匀速
//	                     step_to_run_1--;
//	                     if(step_to_run_1<1)
//	                       status[0]=2;     
//	                   break;
//	              case 2://减速
//	                     count[0]--;
//                       __HAL_TIM_SET_AUTORELOAD(&htim8,period[count[0]]);
//	                    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,period[count[0]]/2);
//	                    if(count[0]<1)
//	                        status[0]=0;
//	                   break;
//	              case 0://停止
//	                   // 关闭通道
//	                  TIM_CCxChannelCmd(TIM8, TIM_CHANNEL_1, TIM_CCx_DISABLE);        
//	                  __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_CC1);
//								
//	                   HAL_TIM_Base_Stop(&htim8);
//									HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_1);
//								__HAL_TIM_SET_COUNTER(&htim8,0);//计数值清零
//								 status[0]=1;//因为是static 在结束完成后，下次运用必须从 status等于0开始
//								
//	                   break;  
//								default: break;//switch 标准
//	          }
//		}
//	}
//	if(htim->Instance==TIM5)

////	 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//	{
//        num_callback[1]++;
//		if(num_callback[1]%2==0)
//		{
//              switch(status[1])
//	          {
//	              case 1://加速
//                       __HAL_TIM_SET_AUTORELOAD(&htim5,period2[count[1]]);
//	                    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,period2[count[1]]/2);
//	                    count[1]++;
//	                    if(count[1]>=ACCELERATED_SPEED_LENGTH2)
//	                    {
//	                        status[1]=3;
//	                    }                        
//	                  break;
//	              case 3://匀速
//	                     step_to_run_2--;
//	                     if(step_to_run_2<1)
//	                       status[1]=2;     
//	                   break;
//	              case 2://减速
//	                     count[1]--;
//                        __HAL_TIM_SET_AUTORELOAD(&htim5,period2[count[1]]);
//	                    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,period2[count[1]]/2);
//	                    if(count[1]<1)
//	                        status[1]=0;
//	                   break;
//	              case 0://停止
//	                   // 关闭通道
//	                  TIM_CCxChannelCmd(TIM5, TIM_CHANNEL_2, TIM_CCx_DISABLE);        
//	                  __HAL_TIM_CLEAR_FLAG(&htim5,TIM_FLAG_CC2);
//	                
//										HAL_TIM_Base_Stop(&htim5);
//										HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_2);
//										__HAL_TIM_SET_COUNTER(&htim5,0);//计数值清零
//									 status[1]=1;//因为是static 在结束完成后，下次运用必须从 status等于0开始
//								
//	                   break;  
//								default: break;//switch 标准
//	          
//	          }
//        }
//		 
//			}
//	stop_flag1=1;
//		
//}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	 static uint32_t count[4]={0,0,0,0};
	static uint32_t num_callback[4]={0,0,0,0};
	static uint8_t status[4]={1,1,1,1};
	
	if(htim->Instance==TIM8)
	{
//		step_to_run_1=2000;
	  stop_flag1=0;		
		num_callback[0]++;
		if(num_callback[0]%2==0)
		{
			switch(status[0])
						{
								case 1://加速
											 __HAL_TIM_SET_AUTORELOAD(&htim8,period[count[0]]);
											__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,period[count[0]]/2);
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
											 __HAL_TIM_SET_AUTORELOAD(&htim8,period[count[0]]);
											__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,period[count[0]]/2);
											if(count[0]<1)
													status[0]=0;
										 break;
								case 0://停止
										 // 关闭通道
										TIM_CCxChannelCmd(TIM8, TIM_CHANNEL_1, TIM_CCx_DISABLE);				
										__HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_CC1);
									//HAL_TIM_Base_Stop(&htim8);
									HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_1);
								__HAL_TIM_SET_COUNTER(&htim8,0);//计数值清零
								 status[0]=1;//因为是static 在结束完成后，下次运用必须从 status等于0开始
								
	                   break;  
								default: break;//switch 标准
										 
						
						}
		}

	
}
if(htim->Instance==TIM5)

//	 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
        num_callback[1]++;
		if(num_callback[1]%2==0)
		{
              switch(status[1])
	          {
	              case 1://加速
                       __HAL_TIM_SET_AUTORELOAD(&htim5,period2[count[1]]);
	                    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,period2[count[1]]/2);
	                    count[1]++;
	                    if(count[1]>=ACCELERATED_SPEED_LENGTH2)
	                    {
	                        status[1]=3;
	                    }                        
	                  break;
	              case 3://匀速
	                     step_to_run_2--;
	                     if(step_to_run_2<1)
	                       status[1]=2;     
	                   break;
	              case 2://减速
	                     count[1]--;
                        __HAL_TIM_SET_AUTORELOAD(&htim5,period2[count[1]]);
	                    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,period2[count[1]]/2);
	                    if(count[1]<1)
	                        status[1]=0;
	                   break;
	              case 0://停止
	                   // 关闭通道
	                  TIM_CCxChannelCmd(TIM5, TIM_CHANNEL_2, TIM_CCx_DISABLE);        
	                  __HAL_TIM_CLEAR_FLAG(&htim5,TIM_FLAG_CC2);
	                
										HAL_TIM_Base_Stop(&htim5);
										HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_2);
										__HAL_TIM_SET_COUNTER(&htim5,0);//计数值清零
									 status[1]=1;//因为是static 在结束完成后，下次运用必须从 status等于0开始
								
	                   break;  
								default: break;//switch 标准
	          
	          }
        }
		 
			}
	stop_flag1++;
	stop_flag2++;
}

void steering_gear(TIM_HandleTypeDef htimx,int TIM_CHANNEL,int y)
{
	 HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL);
	__HAL_TIM_SET_COMPARE(&htimx,TIM_CHANNEL,y);

}
