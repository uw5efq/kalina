/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include <stdio.h>
#include <stdbool.h>

extern uint8_t encoder;
extern uint16_t button_press;
uint8_t status, status_old;
//unsigned long Time, Time_old;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
bool tim_flag = 0;
extern uint16_t count_tach;
extern uint16_t count_tach_over;
extern uint32_t tach;
extern uint16_t count_speed;
extern uint16_t count_speed_over;
extern uint32_t speed;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	if (!(GPIOA->IDR & encoder_scl_Pin) && (!(GPIOB->IDR & encoder_sda_Pin))) {
		status = 0x00;
	} else if ((GPIOA->IDR & encoder_scl_Pin) && (!(GPIOB->IDR & encoder_sda_Pin))) {
		status = 0x10;
	} else if ((GPIOA->IDR & encoder_scl_Pin) && (GPIOB->IDR & encoder_sda_Pin)) {
		status = 0x11;
	} else if (!(GPIOA->IDR & encoder_scl_Pin) && (GPIOB->IDR & encoder_sda_Pin)) {
		status = 0x01;
	}

	if (status_old == 0x10 && status == 0x11) {
		encoder++;
	} else if (status_old == 0x01 && status == 0x00) {
		encoder++;
	}

	if (status_old == 0x11 && status == 0x10) {
		status_old = 0x10;

	} else if (status_old == 0x00 && status == 0x01) {
		status_old = 0x01;
	}

	else if (status_old == 0x10 && status == 0x00) {
		encoder--;
	}

	else if (status_old == 0x01 && status == 0x11) {
		encoder--;
	}
	status_old = status;
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(encoder_sda_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
	count_tach_over++;
  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	count_speed_over++;
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	//таймер 3 дает отсчет в 1 секунду. Работает синхронно с таймером 2. Включает его и выключает.
	count_tach = ((&htim1)->Instance->CNT); //смотрим, чего там натикало в таймере 1
	tach = (count_tach + count_tach_over * ((&htim1)->Instance->ARR) + count_tach_over) * 300; //Вычисляем частоту. было 30 изминил настройку таймера 10
	(&htim1)->Instance->CNT = 0x0000; //сброс счетчика таймера 1.
	count_tach_over = 0; //сброс счетчика переполнения

	//Каждую секунду, он заходит сюда. Здесь мы будет подсчитывать тики, что натикал 2 таймер за эту секунду:
	count_speed = ((&htim2)->Instance->CNT); //смотрим, чего там натикало в таймере 2
	speed = (count_speed + count_speed_over * ((&htim2)->Instance->ARR) + count_speed_over)*300; //Вычисляем частоту.
	(&htim2)->Instance->CNT = 0x0000; //сброс счетчика таймера 2.
	count_speed_over = 0; //сброс счетчика переполнения

	HAL_TIM_Base_Stop_IT(&htim3); //остановим таймер 3
	HAL_TIM_Base_Start_IT(&htim3); //запустим таймер 3
	tim_flag = 1; //дадим разрешение на вывод информации, подняв флаг.
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	if (!(GPIOA->IDR & encoder_scl_Pin) && (!(GPIOB->IDR & encoder_sda_Pin))) {
		status = 0x00;
	} else if ((GPIOA->IDR & encoder_scl_Pin) && (!(GPIOB->IDR & encoder_sda_Pin))) {
		status = 0x10;
	} else if ((GPIOA->IDR & encoder_scl_Pin) && (GPIOB->IDR & encoder_sda_Pin)) {
		status = 0x11;
	} else if (!(GPIOA->IDR & encoder_scl_Pin) && (GPIOB->IDR & encoder_sda_Pin)) {
		status = 0x01;
	}

	if (status_old == 0x10 && status == 0x11) {
		encoder--;
	} else if (status_old == 0x01 && status == 0x00) {
		encoder--;
	}

	if (status_old == 0x11 && status == 0x10) {
		status_old = 0x10;

	} else if (status_old == 0x00 && status == 0x01) {
		status_old = 0x01;
	}

	else if (status_old == 0x10 && status == 0x00) {
		encoder++;
	}
	else if (status_old == 0x01 && status == 0x11) {
		encoder++;
	}
	status_old = status;
	
	if(!(GPIOA->IDR &start_button_in_Pin) && !button_press) {
//		while(!(GPIOA->IDR &start_button_in_Pin)) osDelay(10);
		button_press = 1;
	}
	if((GPIOA->IDR &start_button_in_Pin) && button_press) {
		button_press = 0;
//		while((GPIOA->IDR &start_button_in_Pin)) osDelay(10);
	}
	
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(start_button_in_Pin);
  HAL_GPIO_EXTI_IRQHandler(encoder_scl_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
