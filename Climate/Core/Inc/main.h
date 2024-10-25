/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define led_out_Pin GPIO_PIN_13
#define led_out_GPIO_Port GPIOC
#define damper_Pin GPIO_PIN_0
#define damper_GPIO_Port GPIOA
#define temp_ajust_Pin GPIO_PIN_1
#define temp_ajust_GPIO_Port GPIOA
#define window_airflow_Pin GPIO_PIN_2
#define window_airflow_GPIO_Port GPIOA
#define front_airflof_Pin GPIO_PIN_3
#define front_airflof_GPIO_Port GPIOA
#define on_off_btn_Pin GPIO_PIN_12
#define on_off_btn_GPIO_Port GPIOB
#define auto_btn_Pin GPIO_PIN_13
#define auto_btn_GPIO_Port GPIOB
#define fan_up_btn_Pin GPIO_PIN_14
#define fan_up_btn_GPIO_Port GPIOB
#define fan_down_btn_Pin GPIO_PIN_15
#define fan_down_btn_GPIO_Port GPIOB
#define temp_up_btn_Pin GPIO_PIN_8
#define temp_up_btn_GPIO_Port GPIOA
#define temp_down_btn_Pin GPIO_PIN_9
#define temp_down_btn_GPIO_Port GPIOA
#define fan_Pin GPIO_PIN_4
#define fan_GPIO_Port GPIOB
#define one_wire_Pin GPIO_PIN_6
#define one_wire_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
