/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern ADC_HandleTypeDef hadc1;
extern CRC_HandleTypeDef hcrc;
//extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define REPWR_Pin GPIO_PIN_14
#define REPWR_GPIO_Port GPIOC
#define EN_Pin GPIO_PIN_15
#define EN_GPIO_Port GPIOC
#define SCL_Pin GPIO_PIN_0
#define SCL_GPIO_Port GPIOA
#define SDA_Pin GPIO_PIN_1
#define SDA_GPIO_Port GPIOA
#define ISNS_CH4_Pin GPIO_PIN_4
#define ISNS_CH4_GPIO_Port GPIOA
#define VSNS_CH5_Pin GPIO_PIN_5
#define VSNS_CH5_GPIO_Port GPIOA
#define ISET_T3CH1_Pin GPIO_PIN_6
#define ISET_T3CH1_GPIO_Port GPIOA
#define VSET_T3CH2_Pin GPIO_PIN_7
#define VSET_T3CH2_GPIO_Port GPIOA
#define DET_Pin GPIO_PIN_11
#define DET_GPIO_Port GPIOA
#define DE_Pin GPIO_PIN_12
#define DE_GPIO_Port GPIOA
#define SWD_LED_Pin GPIO_PIN_13
#define SWD_LED_GPIO_Port GPIOA
#define BOOT_SWC_FN_Pin GPIO_PIN_14
#define BOOT_SWC_FN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
