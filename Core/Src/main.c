/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISPLAY_0 SEGA_Pin|SEGB_Pin|SEGC_Pin|SEGD_Pin|SEGE_Pin|SEGF_Pin
#define DISPLAY_1 SEGB_Pin|SEGC_Pin
#define DISPLAY_2 SEGA_Pin|SEGB_Pin|SEGD_Pin|SEGE_Pin|SEGG_Pin
#define DISPLAY_3 SEGA_Pin|SEGB_Pin|SEGC_Pin|SEGD_Pin|SEGG_Pin
#define DISPLAY_4 SEGB_Pin|SEGC_Pin|SEGF_Pin|SEGG_Pin
#define DISPLAY_5 SEGA_Pin|SEGC_Pin|SEGD_Pin|SEGF_Pin|SEGG_Pin
#define DISPLAY_6 SEGA_Pin|SEGC_Pin|SEGD_Pin|SEGE_Pin|SEGF_Pin|SEGG_Pin
#define DISPLAY_7 SEGA_Pin|SEGB_Pin|SEGC_Pin
#define DISPLAY_8 SEGA_Pin|SEGB_Pin|SEGC_Pin|SEGD_Pin|SEGE_Pin|SEGF_Pin|SEGG_Pin
#define DISPLAY_9 SEGA_Pin|SEGB_Pin|SEGC_Pin|SEGD_Pin|SEGF_Pin|SEGG_Pin
#define DISPLAY_ALL SEGA_Pin|SEGB_Pin|SEGC_Pin|SEGD_Pin|SEGE_Pin|SEGF_Pin|SEGG_Pin|SEGP_Pin
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
int CURRENT_NUM = 0;
int STOP_NUM = 12;

//uint16_t DISPLAY_0 = SEGA_Pin|SEGB_Pin|SEGC_Pin|SEGD_Pin|SEGE_Pin|SEGF_Pin;
//uint16_t DISPLAY_1 = SEGB_Pin|SEGC_Pin;
//uint16_t DISPLAY_2 = SEGA_Pin|SEGB_Pin|SEGD_Pin|SEGE_Pin|SEGG_Pin;
//uint16_t DISPLAY_3 = SEGA_Pin|SEGB_Pin|SEGC_Pin|SEGD_Pin|SEGG_Pin;
//uint16_t DISPLAY_4 = SEGB_Pin|SEGC_Pin|SEGF_Pin|SEGG_Pin;
//uint16_t DISPLAY_5 = SEGA_Pin|SEGC_Pin|SEGD_Pin|SEGF_Pin|SEGG_Pin;
//uint16_t DISPLAY_6 = SEGA_Pin|SEGC_Pin|SEGD_Pin|SEGE_Pin|SEGF_Pin|SEGG_Pin;
//uint16_t DISPLAY_7 = SEGA_Pin|SEGC_Pin|SEGC_Pin;
//uint16_t DISPLAY_8 = SEGA_Pin|SEGB_Pin|SEGC_Pin|SEGD_Pin|SEGE_Pin|SEGF_Pin|SEGG_Pin;
//uint16_t DISPLAY_9 = SEGA_Pin|SEGB_Pin|SEGC_Pin|SEGD_Pin|SEGF_Pin|SEGG_Pin;
//uint16_t DISPLAY_ALL = SEGA_Pin|SEGB_Pin|SEGC_Pin|SEGD_Pin|SEGE_Pin|SEGF_Pin|SEGG_Pin|SEGP_Pin;
uint16_t DISPLAY_LIST[]={
    DISPLAY_0,
    DISPLAY_1,
    DISPLAY_2,
    DISPLAY_3,
    DISPLAY_4,
    DISPLAY_5,
    DISPLAY_6,
    DISPLAY_7,
    DISPLAY_8,
    DISPLAY_9
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int getShiwei(int num);
int getGewei(int num);

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);

	

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8000000/1000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 30;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000000/1000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEGF_Pin|SEGE_Pin|SEGD_Pin|SEGC_Pin
                          |SEGB_Pin|SEGA_Pin|DIG2_Pin|DIG1_Pin
                          |SEGP_Pin|SEGG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SEGF_Pin SEGE_Pin SEGD_Pin SEGC_Pin
                           SEGB_Pin SEGA_Pin DIG2_Pin DIG1_Pin
                           SEGP_Pin SEGG_Pin */
  GPIO_InitStruct.Pin = SEGF_Pin|SEGE_Pin|SEGD_Pin|SEGC_Pin
                          |SEGB_Pin|SEGA_Pin|DIG2_Pin|DIG1_Pin
                          |SEGP_Pin|SEGG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int getShiwei(int num){
	return num/10;
}
int getGewei(int num){
	return num%10;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim==&htim1){
		int shiwei = getShiwei(CURRENT_NUM);
		int gewei = getGewei(CURRENT_NUM);
		// 从显示效果可以看出DIG1是十位，DIG2是个�?
		if(HAL_GPIO_ReadPin(DIG1_GPIO_Port,DIG1_Pin)==GPIO_PIN_RESET){
			// 第一位是关闭的，则关闭第二位的显示，�?启第�?位的显示
			HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEGA_GPIO_Port,DISPLAY_ALL,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEGA_GPIO_Port,DISPLAY_LIST[shiwei],GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, GPIO_PIN_SET);	
		}else{
			// 第一位是�?启的，则关闭第一位的显示，开启第二位的显�?
			HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEGA_GPIO_Port,DISPLAY_ALL,GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEGA_GPIO_Port,DISPLAY_LIST[gewei],GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, GPIO_PIN_SET);	
		
		}
	}
	if(htim==&htim2){
		if(CURRENT_NUM<STOP_NUM){
			CURRENT_NUM++;
		}
	}



}

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

