/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "fonts.h"
#include "ssd1306.h"
#include "string.h" 
#include "OneWire.h"

extern uint8_t disp;

extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t TxData[8];
extern uint8_t RxData[8];
extern uint32_t TxMailbox;
extern bool Can_Rx, Can_Tx;

extern char lcd1602_tx_buffer[40];
extern float Temp[MAXDEVICES_ON_THE_BUS];
extern uint16_t pwm, pwm_old;
extern float temp_climate;

extern bool climate_state, climate_auto_state;
extern void climate_on_off();
extern void climate_auto_on_off();

extern void servo_deg(uint8_t deg);
extern uint8_t damper_deg;

bool a = 0;
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId DisplayHandle;
osThreadId AnalogReadHandle;
osThreadId DigitalReadHandle;
osThreadId ButtonReadHandle;
osThreadId OutputsHandle;
osThreadId IdleTaskHandle;
osThreadId CanHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ProctDefaultTask(void const * argument);
void ProctDisplay(void const * argument);
void ProctAnalogRead(void const * argument);
void ProctDigitalRead(void const * argument);
void ProctButtonRead(void const * argument);
void ProctOutputs(void const * argument);
void ProctIdle(void const * argument);
void ProctCan(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, ProctDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Display */
  osThreadDef(Display, ProctDisplay, osPriorityNormal, 0, 256);
  DisplayHandle = osThreadCreate(osThread(Display), NULL);

  /* definition and creation of AnalogRead */
  osThreadDef(AnalogRead, ProctAnalogRead, osPriorityNormal, 0, 128);
  AnalogReadHandle = osThreadCreate(osThread(AnalogRead), NULL);

  /* definition and creation of DigitalRead */
  osThreadDef(DigitalRead, ProctDigitalRead, osPriorityNormal, 0, 128);
  DigitalReadHandle = osThreadCreate(osThread(DigitalRead), NULL);

  /* definition and creation of ButtonRead */
  osThreadDef(ButtonRead, ProctButtonRead, osPriorityNormal, 0, 128);
  ButtonReadHandle = osThreadCreate(osThread(ButtonRead), NULL);

  /* definition and creation of Outputs */
  osThreadDef(Outputs, ProctOutputs, osPriorityNormal, 0, 128);
  OutputsHandle = osThreadCreate(osThread(Outputs), NULL);

  /* definition and creation of IdleTask */
  osThreadDef(IdleTask, ProctIdle, osPriorityNormal, 0, 256);
  IdleTaskHandle = osThreadCreate(osThread(IdleTask), NULL);

  /* definition and creation of Can */
  osThreadDef(Can, ProctCan, osPriorityNormal, 0, 128);
  CanHandle = osThreadCreate(osThread(Can), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_ProctDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ProctDefaultTask */
void ProctDefaultTask(void const * argument)
{
  /* USER CODE BEGIN ProctDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ProctDefaultTask */
}

/* USER CODE BEGIN Header_ProctDisplay */
/**
* @brief Function implementing the Display thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProctDisplay */
void ProctDisplay(void const * argument)
{
  /* USER CODE BEGIN ProctDisplay */
  /* Infinite loop */
  for(;;)
  {
		SSD1306_GotoXY (0,0);
		if(!a){
			sprintf(lcd1602_tx_buffer, "T1   %4.1f C ", Temp[0]);
		}
		else{
			sprintf(lcd1602_tx_buffer, "T2   %4.1f C ", Temp[1]);
		}
		SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
		SSD1306_GotoXY (0,18);
		if(climate_auto_state) sprintf(lcd1602_tx_buffer, "TU A %4.1f C ", temp_climate);
		else sprintf(lcd1602_tx_buffer, "TU M %4.1f C ", temp_climate); 
		SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
		SSD1306_GotoXY (0,36);
		sprintf(lcd1602_tx_buffer, "Fan1   %4d", pwm);
		SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
		SSD1306_UpdateScreen();
		osDelay(125);
    osDelay(1);
  }
  /* USER CODE END ProctDisplay */
}

/* USER CODE BEGIN Header_ProctAnalogRead */
/**
* @brief Function implementing the AnalogRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProctAnalogRead */
void ProctAnalogRead(void const * argument)
{
  /* USER CODE BEGIN ProctAnalogRead */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ProctAnalogRead */
}

/* USER CODE BEGIN Header_ProctDigitalRead */
/**
* @brief Function implementing the DigitalRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProctDigitalRead */
void ProctDigitalRead(void const * argument)
{
  /* USER CODE BEGIN ProctDigitalRead */
  /* Infinite loop */
  for(;;)
  {
		if(climate_state){
//			GPIOC->BRR = led_out_Pin;
			a = 1;
//			servo_deg(0);
			osDelay(1000);
//			GPIOC->BSRR = led_out_Pin;
			a = 0;
//			servo_deg(140);
			osDelay(1000);
			
			get_Temperature();
		}
    osDelay(1);
  }
  /* USER CODE END ProctDigitalRead */
}

/* USER CODE BEGIN Header_ProctButtonRead */
/**
* @brief Function implementing the ButtonRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProctButtonRead */
void ProctButtonRead(void const * argument)
{
  /* USER CODE BEGIN ProctButtonRead */
  /* Infinite loop */
  for(;;)
  {
		if(!(GPIOB->IDR &on_off_btn_Pin)) { // кнопка on_off Климата
			climate_on_off();
			while(!(GPIOB->IDR &on_off_btn_Pin)) osDelay(10);
		}
		if(!(GPIOB->IDR &auto_btn_Pin) && climate_state) { // кнопка auto Климата
			climate_auto_on_off();
			while(!(GPIOB->IDR &auto_btn_Pin)) osDelay(10);
		}		

		if(!(GPIOB->IDR &fan_up_btn_Pin) && climate_state) { // кнопка + вентилятора Климата
			if(pwm<10) pwm++;
			while(!(GPIOB->IDR &fan_up_btn_Pin)) osDelay(10);
		}
		if(!(GPIOB->IDR &fan_down_btn_Pin) && climate_state) { // кнопка - вентилятора Климата
			if(pwm>0)	pwm--;
			while(!(GPIOB->IDR &fan_down_btn_Pin)) osDelay(10);
		}
		
		if(!(GPIOA->IDR &temp_up_btn_Pin)) { // кнопка + температуры Климата
			if(temp_climate<80){
				temp_climate+=0.5;
//				servo_deg(45+temp_climate);
			}
			while(!(GPIOA->IDR &temp_up_btn_Pin)) osDelay(10);
		}
		if(!(GPIOA->IDR &temp_down_btn_Pin)) { // кнопка - температуры Климата
			if(temp_climate>=0.5){
				temp_climate-=0.5;
//				servo_deg(45+temp_climate);
			}
			while(!(GPIOA->IDR &temp_down_btn_Pin)) osDelay(10);
		}		
		osDelay(100);
  }
  /* USER CODE END ProctButtonRead */
}

/* USER CODE BEGIN Header_ProctOutputs */
/**
* @brief Function implementing the Outputs thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProctOutputs */
void ProctOutputs(void const * argument)
{
  /* USER CODE BEGIN ProctOutputs */
  /* Infinite loop */
  for(;;)
  {
		if(climate_state){
			if(pwm != pwm_old){
				if(pwm==10) TIM3->CCR1 = pwm*25+5;
				else TIM3->CCR1 = pwm*25;
//				TIM3->CCR2 = pwm;
				pwm_old = pwm;
			}
			
			if(climate_auto_state){
				if(Temp[1] > temp_climate){
					if(damper_deg > 0){
						damper_deg--;
						servo_deg(damper_deg);
					}
				}
				else{
					if(damper_deg < 180){
						damper_deg++;
						servo_deg(damper_deg);
					}					
				}
				osDelay(3000);
			}
		}
    osDelay(100);
  }
  /* USER CODE END ProctOutputs */
}

/* USER CODE BEGIN Header_ProctIdle */
/**
* @brief Function implementing the IdleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProctIdle */
void ProctIdle(void const * argument)
{
  /* USER CODE BEGIN ProctIdle */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ProctIdle */
}

/* USER CODE BEGIN Header_ProctCan */
/**
* @brief Function implementing the Can thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProctCan */
void ProctCan(void const * argument)
{
  /* USER CODE BEGIN ProctCan */
  /* Infinite loop */
  for(;;)
  {
		if(Can_Rx){ //добавил if проверю как работает
			Can_Rx = 0;
//			if(RxData[2] != 255){
//				switch (RxData[2])
//				{
//					case 10: RxData[2] = 255; mirror_heating_off();
//						break;
//					case 11: RxData[2] = 255; mirror_heating_on();
//						break;
//	//				default:
//	//					break;
//				}
//			}
		}

//		if(Can_Tx){
//			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
//			if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
//			{
//		//		HAL_UART_Transmit(&huart1, (uint8_t*)"ER SEND\n", 8, 100);
//			}
//			TxData[0] = 255;
//			TxData[1] = 255;
//			TxData[2] = 255;
//			Can_Tx = 0;
//		}
	  osDelay(10);
  }
  /* USER CODE END ProctCan */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

