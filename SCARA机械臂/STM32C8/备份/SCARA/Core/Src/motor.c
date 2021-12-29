
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
extern int all_step_1;		//�ܹ����в���
extern int all_step_2;		//�ܹ����в���
extern int all_step_3;
int step_to_run[4]={5000,5000,6800,6800}; //Ҫ�������еĲ���	//Ҫ�������еĲ���			 �ܹ����в��� = ACCELERATED_SPEED_LENGTH*2 + step_to_run	 
int step_to_run_1=600;
int	 step_to_run_2=3000;
int	 step_to_run_3=1000;
int step_to_run_4=1000;

 //����˵���������ÿһ������ļ���ֵ
void CalculateSModelLine(float fre_[], unsigned short period_[], float len, float fre_max, float fre_min, float flexible)
{	 
		step_to_run_1=all_step_1-100;
	 step_to_run_2=all_step_2-100;
	 step_to_run_3=all_step_3-100;
		int i=0;
		float deno ;
		float melo ;
		float delt = fre_max-fre_min;
		for(; i<len; i++)
		{
				melo = flexible* (i-len/2) / (len/2);	// xֵ
				deno = 1.0f / (1 + expf(-melo));	//expf is a library function of exponential(e)?	 yֵ
				fre_[i] = delt * deno + fre_min;
				period_[i] = (unsigned short)(4500000.0f / fre[i]); // 10000000 is the timer driver frequency
		}
		
}

//	void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	 static uint32_t count[4]={0,0,0,0};
//	static uint32_t num_callback[4]={0,0,0,0};
//	static uint8_t status[4]={1,1,1,1};
//	
//	if(htim->Instance==TIM1)
//	{
////		step_to_run_1=2000;
//	  stop_flag1=0;		
//		num_callback[0]++;
//		if(num_callback[0]%2==0)
//		{
//			switch(status[0])
//						{
//								case 1://����
//											 __HAL_TIM_SET_AUTORELOAD(&htim1,period[count[0]]);
//											__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,period[count[0]]/2);
//											count[0]++;
//											if(count[0]>=ACCELERATED_SPEED_LENGTH1)
//											{
//													status[0]=3;
//											}												
//										break;
//								case 3://����
//											 step_to_run_1--;
//											 if(step_to_run_1<1)
//												 status[0]=2;		 
//										 break;
//								case 2://����
//											 count[0]--;
//											 __HAL_TIM_SET_AUTORELOAD(&htim1,period[count[0]]);
//											__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,period[count[0]]/2);
//											if(count[0]<1)
//													status[0]=0;
//										 break;
//								case 0://ֹͣ
//										 // �ر�ͨ��
//										TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_DISABLE);				
//										__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
//										 break;
//						
//						}
//		}
//		 __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_CC1);
//	   status[0]=1;
//		stop_flag1=1;
//}

//if(htim->Instance==TIM2)
//{
//				stop_flag2=0;	
//				num_callback[2]++;
////			 step_to_run_2=4000;
//		if(num_callback[2]%2==0)
//		{
//							switch(status[2])
//						{
//								case 1://����
//											 __HAL_TIM_SET_AUTORELOAD(&htim2,period[count[2]]);
//											__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,period[count[2]]/2);
//											count[2]++;
//											if(count[2]>=ACCELERATED_SPEED_LENGTH2)
//											{
//													status[2]=3;
//											}												
//										break;
//								case 3://����
//											 step_to_run_4--;
//											 if(step_to_run_4<1)
//												 status[2]=2;		 
//										 break;
//								case 2://����
//											 count[2]--;
//												__HAL_TIM_SET_AUTORELOAD(&htim2,period[count[2]]);
//											__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,period[count[2]]/2);
//											if(count[2]<1)
//													status[2]=0;
//										 break;
//								case 0://ֹͣ
//										 // �ر�ͨ��
//										TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_DISABLE);				
//										__HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC1);
//										 break;
//						
//						}
//				}
//		 __HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC1);
//		
//		stop_flag2=1;
//			}
//		status[2]=1;

//if(htim->Instance==TIM3)
//{
//				stop_flag3=0;	
//				num_callback[3]++;
//		if(num_callback[3]%2==0)
//		{
//							switch(status[3])
//						{
//								case 1://����
//											 __HAL_TIM_SET_AUTORELOAD(&htim3,period[count[3]]);
//											__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,period[count[3]]/2);
//											count[3]++;
//											if(count[3]>=ACCELERATED_SPEED_LENGTH2)
//											{
//													status[3]=3;
//											}												
//										break;
//								case 3://����
//											 step_to_run_4--;
//											 if(step_to_run_4<1)
//												 status[3]=2;		 
//										 break;
//								case 2://����
//											 count[3]--;
//												__HAL_TIM_SET_AUTORELOAD(&htim3,period[count[3]]);
//											__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,period[count[3]]/2);
//											if(count[3]<1)
//													status[3]=0;
//										 break;
//								case 0://ֹͣ
//										 // �ر�ͨ��
//										TIM_CCxChannelCmd(TIM3, TIM_CHANNEL_1, TIM_CCx_DISABLE);				
//										__HAL_TIM_CLEAR_FLAG(&htim3,TIM_FLAG_CC1);
//										 break;
//						
//						}
//				}
//		 __HAL_TIM_CLEAR_FLAG(&htim3,TIM_FLAG_CC1);
//			}
//	status[3]=1;
//	stop_flag3=1;
//}
//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	 static uint32_t count[4]={0,0,0,0};
//	static uint32_t num_callback[4]={0,0,0,0};
//	static uint8_t status[4]={1,1,1,1};

//	if(htim->Instance==TIM1)
//	{	
//		num_callback[0]++;
//		if(num_callback[0]%2==0)
//		{
//			switch(status[0])
//						{
//								case 1://����
//											 __HAL_TIM_SET_AUTORELOAD(&htim1,period[count[0]]);
//											__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,period[count[0]]/2);
//											count[0]++;
//											if(count[0]>=ACCELERATED_SPEED_LENGTH1)
//											{
//													status[0]=3;
//											}												
//										break;
//								case 3://����
//											 step_to_run_1--;
//											 if(step_to_run_1<1)
//												 status[0]=2;		 
//										 break;
//								case 2://����
//											 count[0]--;
//											 __HAL_TIM_SET_AUTORELOAD(&htim1,period[count[0]]);
//											__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,period[count[0]]/2);
//											if(count[0]<1)
//													status[0]=0;
//										 break;
//								case 0://ֹͣ
//										 // �ر�ͨ��
//										TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_DISABLE);				
//										__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
//									//HAL_TIM_Base_Stop(&htim8);
//									HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
//								__HAL_TIM_SET_COUNTER(&htim1,0);//����ֵ����
//								 status[0]=1;//��Ϊ��static �ڽ�����ɺ��´����ñ���� status����0��ʼ
//								
//	                   break;  
//								default: break;//switch ��׼
//						}
//		}
//}
//if(htim->Instance==)
//	{
//     num_callback[1]++;
//		if(num_callback[1]%2==0)
//		{
//              switch(status[1])
//	          {
//	              case 1://����
//                       __HAL_TIM_SET_AUTORELOAD(&htim2,period2[count[1]]);
//	                    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,period2[count[1]]/2);
//	                    count[1]++;
//	                    if(count[1]>=ACCELERATED_SPEED_LENGTH2)
//	                    {
//	                        status[1]=3;
//	                    }                        
//	                  break;
//	              case 3://����
//	                     step_to_run_2--;
//	                     if(step_to_run_2<1)
//	                       status[1]=2;     
//	                   break;
//	              case 2://����
//	                     count[1]--;
//                        __HAL_TIM_SET_AUTORELOAD(&htim2,period2[count[1]]);
//	                    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,period2[count[1]]/2);
//	                    if(count[1]<1)
//	                        status[1]=0;
//	                   break;
//	              case 0://ֹͣ
//	                   // �ر�ͨ��
//	                  TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_DISABLE);        
//	                  __HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC1);
////										HAL_TIM_Base_Stop(&htim2);
////										HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
////										__HAL_TIM_SET_COUNTER(&htim2,0);//����ֵ����
//										status[1]=1;//��Ϊ��static �ڽ�����ɺ��´����ñ���� status����0��ʼ
//										break;      
//								default: break;//switch ��׼
//	          
//	          }
//        }
//	}
//if(htim->Instance==TIM4)
//	{
//        num_callback[2]++;
//		if(num_callback[2]%2==0)
//		{
//              switch(status[2])
//	          {
//	              case 1://����
//                       __HAL_TIM_SET_AUTORELOAD(&htim4,period2[count[2]]);
//	                    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,period2[count[2]]/2);
//	                    count[2]++;
//	                    if(count[2]>=ACCELERATED_SPEED_LENGTH2)
//	                    {
//	                        status[2]=3;
//	                    }                        
//	                  break;
//	              case 3://����
//	                     step_to_run_3--;
//	                     if(step_to_run_3<1)
//	                       status[2]=2;     
//	                   break;
//	              case 2://����
//	                     count[1]--;
//                        __HAL_TIM_SET_AUTORELOAD(&htim4,period2[count[1]]);
//	                    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,period2[count[2]]/2);
//	                    if(count[2]<1)
//	                        status[2]=0;
//	                   break;
//	              case 0://ֹͣ
//	                   // �ر�ͨ��
//	                  TIM_CCxChannelCmd(TIM4, TIM_CHANNEL_3, TIM_CCx_DISABLE);        
//	                  __HAL_TIM_CLEAR_FLAG(&htim4,TIM_FLAG_CC2);
//	                
//										HAL_TIM_Base_Stop(&htim4);
//										HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_3);
//										__HAL_TIM_SET_COUNTER(&htim4,0);//����ֵ����
//									 status[2]=1;//��Ϊ��static �ڽ�����ɺ��´����ñ���� status����0��ʼ
//								
//	                   break;  
//								default: break;//switch ��׼
//	          
//	          }
//        }
//		 
//			}		
//}

//		
//motor_result(-150,80);
//	CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH1,FRE_MAX,FRE_MIN,4);
//	CalculateSModelLine(fre2,period2,ACCELERATED_SPEED_LENGTH2,FRE_MAX,FRE_MIN,4);
//	CalculateSModelLine(fre3,period2,ACCELERATED_SPEED_LENGTH2,FRE_MAX,FRE_MIN,4);
//	HAL_GPIO_WritePin(GPIOC, EN4_Pin|EN3_Pin|EN2_Pin|EN1_Pin, GPIO_PIN_RESET);

//	//CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH,FRE_MAX,FRE_MIN,4);
//  HAL_TIM_Base_Start(&htim1);//������ʱ��
//	HAL_TIM_Base_Start(&htim2);//������ʱ��
//	HAL_TIM_Base_Start(&htim3);//������ʱ��
//	HAL_TIM_Base_Start(&htim4);//������ʱ��
//	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);//������ʱ��8ͨ��1�Ƚ�����ж�
//	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);//������ʱ��8ͨ��2�Ƚ�����ж�
//	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);//������ʱ��8ͨ��1�Ƚ�����ж�
//	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_3);//������ʱ��8ͨ��2�Ƚ�����ж�
////	
//  	HAL_Delay(3000);
////	 HAL_GPIO_WritePin(GPIOB, DIR2_Pin, GPIO_PIN_SET);//DIR2 ��۸ߵ�ƽ��ʱ��ת��
//// HAL_GPIO_WritePin(GPIOB, DIR3_Pin, GPIO_PIN_SET);//DIR3 С�۸ߵ�ƽ��ʱ��ת��
//		motor_result(120,80);
//	CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH1,FRE_MAX,FRE_MIN,4);
//	CalculateSModelLine(fre2,period2,ACCELERATED_SPEED_LENGTH2,FRE_MAX,FRE_MIN,4);
//	CalculateSModelLine(fre3,period2,ACCELERATED_SPEED_LENGTH2,FRE_MAX,FRE_MIN,4);
//	HAL_GPIO_WritePin(GPIOC, EN4_Pin|EN3_Pin|EN2_Pin|EN1_Pin, GPIO_PIN_RESET);
//  HAL_TIM_Base_Start(&htim1);//������ʱ��
//	HAL_TIM_Base_Start(&htim2);//������ʱ��
//	HAL_TIM_Base_Start(&htim3);//������ʱ��
//	HAL_TIM_Base_Start(&htim4);//������ʱ��
//	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);//������ʱ��8ͨ��1�Ƚ�����ж�
//	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);//������ʱ��8ͨ��2�Ƚ�����ж�
//	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);//������ʱ��8ͨ��1�Ƚ�����ж�
//	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_3);//������ʱ��8ͨ��2�Ƚ�����ж�
//	
//	HAL_Delay(3000);
//	
//	motor_result(20,20);
//	CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH1,FRE_MAX,FRE_MIN,4);
//	CalculateSModelLine(fre2,period2,ACCELERATED_SPEED_LENGTH2,FRE_MAX,FRE_MIN,4);
//	CalculateSModelLine(fre3,period2,ACCELERATED_SPEED_LENGTH2,FRE_MAX,FRE_MIN,4);
//	HAL_GPIO_WritePin(GPIOC, EN4_Pin|EN3_Pin|EN2_Pin|EN1_Pin, GPIO_PIN_RESET);
//  HAL_TIM_Base_Start(&htim1);//������ʱ��
//	HAL_TIM_Base_Start(&htim2);//������ʱ��
//	HAL_TIM_Base_Start(&htim3);//������ʱ��
//	HAL_TIM_Base_Start(&htim4);//������ʱ��
//	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);//������ʱ��8ͨ��2�Ƚ�����ж�
//	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);//������ʱ��8ͨ��1�Ƚ�����ж�
//	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);//������ʱ��8ͨ��1�Ƚ�����ж�
//	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_3);//������ʱ��8ͨ��2�Ƚ�����ж�
//	if(stop_flag1&stop_flag2& stop_flag3==1)
//	{
//		steering_gear(htim5,TIM_CHANNEL_1,100);
//	}
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
								case 1://����
											 __HAL_TIM_SET_AUTORELOAD(&htim1,period[count[0]]);
											__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,period[count[0]]/2);
											count[0]++;
											if(count[0]>=ACCELERATED_SPEED_LENGTH1)
											{
													status[0]=3;
											}												
										break;
								case 3://����
											 step_to_run_1--;
											 if(step_to_run_1<1)
												 status[0]=2;		 
										 break;
								case 2://����
											 count[0]--;
											 __HAL_TIM_SET_AUTORELOAD(&htim1,period[count[0]]);
											__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,period[count[0]]/2);
											if(count[0]<1)
													status[0]=0;
										 break;
								case 0://ֹͣ
										 // �ر�ͨ��
										TIM_CCxChannelCmd(TIM8, TIM_CHANNEL_1, TIM_CCx_DISABLE);				
										__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
									HAL_TIM_Base_Stop(&htim1);
									HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
								__HAL_TIM_SET_COUNTER(&htim1,0);//����ֵ����
								 status[0]=1;//��Ϊ��static �ڽ�����ɺ��´����ñ���� status����0��ʼ
								
	                   break;  
								default: break;//switch ��׼
										 
						
						}
		}

	
}
if(htim->Instance==TIM2)

//	 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
        num_callback[1]++;
		if(num_callback[1]%2==0)
		{
              switch(status[1])
	          {
	              case 1://����
                       __HAL_TIM_SET_AUTORELOAD(&htim2,period2[count[1]]);
	                    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,period2[count[1]]/2);
	                    count[1]++;
	                    if(count[1]>=ACCELERATED_SPEED_LENGTH2)
	                    {
	                        status[1]=3;
	                    }                        
	                  break;
	              case 3://����
	                     step_to_run_2--;
	                     if(step_to_run_2<1)
	                       status[1]=2;     
	                   break;
	              case 2://����
	                     count[1]--;
                        __HAL_TIM_SET_AUTORELOAD(&htim2,period2[count[1]]);
	                    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,period2[count[1]]/2);
	                    if(count[1]<1)
	                        status[1]=0;
	                   break;
	              case 0://ֹͣ
	                   // �ر�ͨ��
	                  TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_DISABLE);        
	                  __HAL_TIM_CLEAR_FLAG(&htim5,TIM_FLAG_CC1);
	                
										HAL_TIM_Base_Stop(&htim2);
										HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
										__HAL_TIM_SET_COUNTER(&htim2,0);//����ֵ����
									 status[1]=1;//��Ϊ��static �ڽ�����ɺ��´����ñ���� status����0��ʼ
								
	                   break;  
								default: break;//switch ��׼
	          
	          }
        }
		 
			}
		
if(htim->Instance==TIM4)
	{	
		num_callback[2]++;
		if(num_callback[2]%2==0)
		{
			switch(status[2])
						{
								case 1://����
											 __HAL_TIM_SET_AUTORELOAD(&htim4,period[count[2]]);
											__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,period[count[2]]/2);
											count[2]++;
											if(count[2]>=ACCELERATED_SPEED_LENGTH1)
											{
													status[2]=3;
											}												
										break;
								case 3://����
											 step_to_run_3--;
											 if(step_to_run_3<1)
												 status[2]=2;		 
										 break;
								case 2://����
											 count[2]--;
											 __HAL_TIM_SET_AUTORELOAD(&htim4,period[count[2]]);
											__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,period[count[2]]/2);
											if(count[2]<1)
													status[2]=0;
										 break;
								case 0://ֹͣ
										 // �ر�ͨ��
										TIM_CCxChannelCmd(TIM4, TIM_CHANNEL_3, TIM_CCx_DISABLE);				
										__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_CC3);
									HAL_TIM_Base_Stop(&htim4);
									HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_3);
								__HAL_TIM_SET_COUNTER(&htim4,0);//����ֵ����
								 status[2]=1;//��Ϊ��static �ڽ�����ɺ��´����ñ���� status����0��ʼ
								
	                   break;  
								default: break;//switch ��׼
						}
		
}
}
}

void steering_gear(TIM_HandleTypeDef htimx,int TIM_CHANNEL,int y)
{
	 HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL);
	__HAL_TIM_SET_COMPARE(&htimx,TIM_CHANNEL,y);

}
