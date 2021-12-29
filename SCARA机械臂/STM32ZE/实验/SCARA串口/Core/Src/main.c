/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "robot.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int uart_flag;
int uart_flag1;

uint8_t x1,y1,x2,y2;
extern int  stop_flag1;
extern int  stop_flag2;
float fre[ACCELERATED_SPEED_LENGTH1]; //����洢���ٹ�����ÿһ����Ƶ�� 
unsigned short period[ACCELERATED_SPEED_LENGTH1]; //���鴢����ٹ�����ÿһ����ʱ�����Զ�װ��ֵ 

float fre2[ACCELERATED_SPEED_LENGTH2]; //����洢���ٹ�����ÿһ����Ƶ�� 
unsigned short period2[ACCELERATED_SPEED_LENGTH2]; //���鴢����ٹ�����ÿһ����ʱ�����Զ�װ��ֵ
 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	int fputc(int ch, FILE *f);
	int fgetc(FILE *f);
	void steering_gear(TIM_HandleTypeDef htimx,int TIM_CHANNEL,int y);//���
	void robot_arm(float x,float y);//����
	void motor_result(float x,float y);//����ó��������
	void motor_reset();//��λ����
	void CalculateSModelLine(float fre_[], unsigned short period_[], float len, float fre_max, float fre_min, float flexible);
	
	//���ڽ����жϻص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance ==USART1)
	{
		uint8_t temp;
		uart_flag=(uart_flag+1)%5;
		if(uart_flag==1)
		{
			printf("����ץȡ����y1\r\n");
			HAL_UART_Receive_IT(&huart1,&y1,1);
		}
		if(uart_flag==2)
		{
			printf("�����������x2\r\n");
			HAL_UART_Receive_IT(&huart1,&x2,1);
		}
		if(uart_flag==3)
		{
			printf("�����������y2\r\n");
			HAL_UART_Receive_IT(&huart1,&y2,1);
		}
		if(uart_flag==4)
		{
			HAL_UART_Receive_IT(&huart1,&temp,1);
			uart_flag1=1;
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	//EN2ʹ��
	
	printf("����ץȡ����x1\r\n");
	HAL_UART_Receive_IT(&huart1,&x1,1);
	HAL_Delay(1000);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

			
    /* USER CODE BEGIN 3 */
		if(uart_flag1==1)
	{
	printf("x1Ϊ%d\r\n",x1);
	printf("y1Ϊ%d\r\n",y1);
	printf("x2Ϊ%d\r\n",x2);
	printf("y2Ϊ%d\r\n",y2);
	HAL_Delay(1000);
	steering_gear(htim3,TIM_CHANNEL_1,10);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
	motor_result(x1,y1);
	CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH1,FRE_MAX,FRE_MIN,4);
	CalculateSModelLine(fre2,period2,ACCELERATED_SPEED_LENGTH2,FRE_MAX,FRE_MIN,4);
	//CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH,FRE_MAX,FRE_MIN,4);
  HAL_TIM_Base_Start(&htim8);//������ʱ��
	HAL_TIM_Base_Start(&htim5);//������ʱ��
	HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);//������ʱ��8ͨ��1�Ƚ�����ж�
	HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_2);//������ʱ��8ͨ��2�Ƚ�����ж�
	HAL_Delay(3000);
	steering_gear(htim3,TIM_CHANNEL_1,25);
	HAL_Delay(500);
	if(stop_flag1==1)
	{
		motor_result(x2,y2);
		CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH1,FRE_MAX,FRE_MIN,4);
		CalculateSModelLine(fre2,period2,ACCELERATED_SPEED_LENGTH2,FRE_MAX,FRE_MIN,4);
		//CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH,FRE_MAX,FRE_MIN,4);
		HAL_TIM_Base_Start(&htim8);//������ʱ��
		HAL_TIM_Base_Start(&htim5);//������ʱ��
		HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);//������ʱ��8ͨ��1�Ƚ�����ж�
		HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_2);//������ʱ��8ͨ��2�Ƚ�����ж�
		HAL_Delay(1000);
	}
		HAL_Delay(800);
		steering_gear(htim3,TIM_CHANNEL_1,15);
		HAL_Delay(500);
		motor_result(30,10);
		CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH1,FRE_MAX,FRE_MIN,4);
		CalculateSModelLine(fre2,period2,ACCELERATED_SPEED_LENGTH2,FRE_MAX,FRE_MIN,4);
		HAL_TIM_Base_Start(&htim8);//������ʱ��
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
//	HAL_TIM_Base_Start(&htim5);//������ʱ��
		HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);//������ʱ��8ͨ��1�Ƚ�����ж�
//	HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_2);//������ʱ��8ͨ��2�Ƚ�����ж�
		HAL_Delay(1000);
		HAL_Delay(1000);
	uart_flag1=0;
}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
