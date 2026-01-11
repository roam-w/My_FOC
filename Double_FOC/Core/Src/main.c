/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AS5600.h"
#include "pid.h"
#include "current_sense.h"
#include "MyFOC.h"
#include "common.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
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
Pid_HandleTypedef hpid_speed;
Pid_HandleTypedef hpid_torque;

CurrentSense_HandleTypeDef current_u, current_v;

AS5600_HandleTypeDef has5600;
float angle = 0;
float motor_target = 60;

 float open_loop_timestamp = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	// 启动定时器
	HAL_TIM_Base_Start(&htim2);

	// 初始化DMA接收
	UART2_DMA_Init_Receive();
	
	/* 初始化AS5600 */
	AS5600_Init(&has5600, &hi2c1);
	
	//PID初始化 力-速串级pid
	Pid_Init(&hpid_torque, 0.01, 0.01, 0, 6, 100000);
	Pid_Init(&hpid_speed, 0.013, 0.02, 0, 6, 100000);
//	
//	// 电流检测初始化
//	CurrentSense_Init(&current_u, &hadc1, ADC_CHANNEL_0, 0.01f);
//	CurrentSense_Init(&current_v, &hadc1, ADC_CHANNEL_1, 0.01f);
//	
//	//零漂校准
//	AS5600_CalibrateZero(&h_as5600);
//	CurrentSense_CalibrateOffset(&current_u, 100);
//	CurrentSense_CalibrateOffset(&current_v, 100);
	
	// 启动PWM通道
  HAL_GPIO_WritePin(PWM_EN_GPIO_Port, PWM_EN_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);	
	
//	UART_DMA_Send(&huart2, "Hello World!\r\n", strlen("Hello World!\r\n"));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		float angle_r = AS5600_GetRadian(&has5600);

////		setPhaseVoltage(_constrain(Kp*(motor_target-(-1)*angle), -6, 6), 0, _electricAngle(angle_r));	// 位置闭环
//		setPhaseVoltage(motor_target, 0, _electricAngle(angle_r));	//电压力矩闭环
//		
		float velocity = AS5600_GetVelocity(&has5600);		
		float back = Pid_Output(&hpid_speed, motor_target, (-1.0f)*velocity);
		setPhaseVoltage(_constrain(back, -6, 6), 0, _electricAngle(angle_r));		// 速度闭环

		if (UART2_DMA_Get_Received_Length() > 0)
		{
			uint16_t len = UART2_DMA_Get_Received_Length();
			uint8_t *rx_data = UART2_DMA_Get_Receive_Buffer();
			  
			((char*)rx_data)[len] = '\0';
			
			motor_target = (float)atof((char*)rx_data);
			
			UART2_DMA_Transmit((uint8_t*)&motor_target, sizeof(motor_target));
			// 准备下一次接收
			UART2_DMA_Start_Receive();
		}
		
		char buffer[8];
		sprintf(buffer, "%f", velocity);
		UART2_DMA_Wait_Transmit_Complete();
		UART2_DMA_Transmit((uint8_t*)buffer, sizeof(buffer));
		UART2_DMA_Wait_Transmit_Complete();
		UART2_DMA_Transmit("\r\n", strlen("\r\n"));
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
