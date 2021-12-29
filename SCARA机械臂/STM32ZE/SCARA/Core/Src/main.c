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
extern int  stop_flag1;
float fre[ACCELERATED_SPEED_LENGTH1]; //数组存储加速过程中每一步的频率 
unsigned short period[ACCELERATED_SPEED_LENGTH1]; //数组储存加速过程中每一步定时器的自动装载值 

float fre2[ACCELERATED_SPEED_LENGTH2]; //数组存储加速过程中每一步的频率 
unsigned short period2[ACCELERATED_SPEED_LENGTH2]; //数组储存加速过程中每一步定时器的自动装载值
 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	void steering_gear(TIM_HandleTypeDef htimx,int TIM_CHANNEL,int y);//舵机
	void robot_arm(float x,float y);//解算
	void motor_result(float x,float y);//计算得出相对坐标
	void motor_reset();//复位程序
	void CalculateSModelLine(float fre_[], unsigned short period_[], float len, float fre_max, float fre_min, float flexible);
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
	//EN2使能



	steering_gear(htim3,TIM_CHANNEL_1,10);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
	motor_result(120,60);
	CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH1,FRE_MAX,FRE_MIN,4);
	CalculateSModelLine(fre2,period2,ACCELERATED_SPEED_LENGTH2,FRE_MAX,FRE_MIN,4);
	//CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH,FRE_MAX,FRE_MIN,4);
  HAL_TIM_Base_Start(&htim8);//启动定时器
	HAL_TIM_Base_Start(&htim5);//启动定时器
	HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);//启动定时器8通道1比较输出中断
	HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_2);//启动定时器8通道2比较输出中断
	HAL_Delay(3000);
	steering_gear(htim3,TIM_CHANNEL_1,25);
	HAL_Delay(100);
	if(stop_flag1==1)
	{
		motor_result(100,100);
		CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH1,FRE_MAX,FRE_MIN,4);
		CalculateSModelLine(fre2,period2,ACCELERATED_SPEED_LENGTH2,FRE_MAX,FRE_MIN,4);
		//CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH,FRE_MAX,FRE_MIN,4);
		HAL_TIM_Base_Start(&htim8);//启动定时器
		HAL_TIM_Base_Start(&htim5);//启动定时器
		HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);//启动定时器8通道1比较输出中断
		HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_2);//启动定时器8通道2比较输出中断
		HAL_Delay(1000);
	}
	HAL_Delay(800);
		steering_gear(htim3,TIM_CHANNEL_1,15);
	
		HAL_Delay(1000);
	if(stop_flag1==1)
	{
		
		motor_result(30,10);
		CalculateSModelLine(fre,period,ACCELERATED_SPEED_LENGTH1,FRE_MAX,FRE_MIN,4);
		CalculateSModelLine(fre2,period2,ACCELERATED_SPEED_LENGTH2,FRE_MAX,FRE_MIN,4);
		HAL_TIM_Base_Start(&htim8);//启动定时器
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
//		HAL_TIM_Base_Start(&htim5);//启动定时器
		HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);//启动定时器8通道1比较输出中断
//		HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_2);//启动定时器8通道2比较输出中断
		HAL_Delay(1000);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
