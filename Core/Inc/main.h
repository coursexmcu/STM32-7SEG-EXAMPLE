/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SEGF_Pin GPIO_PIN_10
#define SEGF_GPIO_Port GPIOB
#define SEGE_Pin GPIO_PIN_11
#define SEGE_GPIO_Port GPIOB
#define SEGD_Pin GPIO_PIN_12
#define SEGD_GPIO_Port GPIOB
#define SEGC_Pin GPIO_PIN_13
#define SEGC_GPIO_Port GPIOB
#define SEGB_Pin GPIO_PIN_14
#define SEGB_GPIO_Port GPIOB
#define SEGA_Pin GPIO_PIN_15
#define SEGA_GPIO_Port GPIOB
#define DIG2_Pin GPIO_PIN_6
#define DIG2_GPIO_Port GPIOB
#define DIG1_Pin GPIO_PIN_7
#define DIG1_GPIO_Port GPIOB
#define SEGP_Pin GPIO_PIN_8
#define SEGP_GPIO_Port GPIOB
#define SEGG_Pin GPIO_PIN_9
#define SEGG_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
