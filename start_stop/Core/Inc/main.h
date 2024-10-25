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
#define led_out_Pin GPIO_PIN_13
#define led_out_GPIO_Port GPIOC
#define starter_rele_out_Pin GPIO_PIN_15
#define starter_rele_out_GPIO_Port GPIOC
#define starter_out_Pin GPIO_PIN_1
#define starter_out_GPIO_Port GPIOA
#define ignition_out_Pin GPIO_PIN_2
#define ignition_out_GPIO_Port GPIOA
#define immo_out_Pin GPIO_PIN_3
#define immo_out_GPIO_Port GPIOA
#define acc_out_Pin GPIO_PIN_4
#define acc_out_GPIO_Port GPIOA
#define brake_in_Pin GPIO_PIN_5
#define brake_in_GPIO_Port GPIOA
#define fuel_level_in_Pin GPIO_PIN_7
#define fuel_level_in_GPIO_Port GPIOA
#define temperature_in_Pin GPIO_PIN_0
#define temperature_in_GPIO_Port GPIOB
#define voltage_in_Pin GPIO_PIN_1
#define voltage_in_GPIO_Port GPIOB
#define hand_brake_in_Pin GPIO_PIN_12
#define hand_brake_in_GPIO_Port GPIOB
#define neutral_in_Pin GPIO_PIN_13
#define neutral_in_GPIO_Port GPIOB
#define oil_lamp_in_Pin GPIO_PIN_14
#define oil_lamp_in_GPIO_Port GPIOB
#define batt_lamp_in_Pin GPIO_PIN_15
#define batt_lamp_in_GPIO_Port GPIOB
#define engine_lamp_in_Pin GPIO_PIN_8
#define engine_lamp_in_GPIO_Port GPIOA
#define immo_lamp_in_Pin GPIO_PIN_9
#define immo_lamp_in_GPIO_Port GPIOA
#define fuel_pump_in_Pin GPIO_PIN_10
#define fuel_pump_in_GPIO_Port GPIOA
#define start_button_in_Pin GPIO_PIN_11
#define start_button_in_GPIO_Port GPIOA
#define start_button_in_EXTI_IRQn EXTI15_10_IRQn
#define encoder_scl_Pin GPIO_PIN_15
#define encoder_scl_GPIO_Port GPIOA
#define encoder_scl_EXTI_IRQn EXTI15_10_IRQn
#define encoder_sda_Pin GPIO_PIN_3
#define encoder_sda_GPIO_Port GPIOB
#define encoder_sda_EXTI_IRQn EXTI3_IRQn
#define clutch_in_Pin GPIO_PIN_4
#define clutch_in_GPIO_Port GPIOB
#define autostart_in_Pin GPIO_PIN_5
#define autostart_in_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
