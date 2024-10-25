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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
//#include "ssd1306.h"
#include "string.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define fr_motor_current 12

#define glass_Stop 0
#define glass_Up 1
#define glass_Down 2
#define glass_btn_time 15

//#define fr_glass_Stop 20
//#define fr_glass_Up 21
//#define fr_glass_Down 22

#define glass_closer_On 23
#define glass_closer_Ok 24

#define glass_blocking_On 25
#define glass_blocking_Off 26

//#define mirror_heating_On 21
//#define mirror_heating_Off 20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c2;

osThreadId defaultTaskHandle;
osThreadId DisplayHandle;
osThreadId ButtonReadHandle;
osThreadId OutputsHandle;
osThreadId BacklightHandle;
osThreadId FoldingMirrorsHandle;
osThreadId CanHandle;
/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = {255, 255, 255,};
uint8_t RxData[8] = {255, 255, 255,};
uint32_t TxMailbox = 0;
#define can_addr_door_fl 0x0001
#define can_addr_door_fr 0x0002
bool Can_Rx = 0;

bool adc1_flag = 0;
float adc1_value = 0, current = 0;

uint8_t fr_glass = 0;

bool glass_blocking = 0, glass_closer_on = 0;
uint16_t fr_glass_btn_time = 0;
uint16_t btn_click_up = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
void StartDefaultTask(void const * argument);
void ProctDisplay(void const * argument);
void ProctButtonRead(void const * argument);
void ProctOutputs(void const * argument);
void ProctBacklight(void const * argument);
void ProctFoldingMirrors(void const * argument);
void ProcCan(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if(hadc->Instance == ADC1){
		adc1_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
		current = (adc1_value*40.0f)/4095;
		adc1_flag = 1;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
		if(RxHeader.StdId == can_addr_door_fr) Can_Rx = 1;
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t er = HAL_CAN_GetError(hcan);
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
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc1);

	TxHeader.StdId = can_addr_door_fl;
	TxHeader.ExtId = 0;
	TxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
	TxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = 0;

	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);

	GPIOA->BRR = glass_up_Pin; // реле подьема стекла выкл
	GPIOA->BRR = glass_down_Pin; // реле подьема стекла выкл
	GPIOA->BRR = open_the_door_Pin; // реле открытия двери выкл
	GPIOA->BRR = mirror_heating_Pin; // реле обогрева зеркал выкл
  /* USER CODE END 2 */

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Display */
  osThreadDef(Display, ProctDisplay, osPriorityNormal, 0, 256);
  DisplayHandle = osThreadCreate(osThread(Display), NULL);

  /* definition and creation of ButtonRead */
  osThreadDef(ButtonRead, ProctButtonRead, osPriorityNormal, 0, 128);
  ButtonReadHandle = osThreadCreate(osThread(ButtonRead), NULL);

  /* definition and creation of Outputs */
  osThreadDef(Outputs, ProctOutputs, osPriorityIdle, 0, 128);
  OutputsHandle = osThreadCreate(osThread(Outputs), NULL);

  /* definition and creation of Backlight */
  osThreadDef(Backlight, ProctBacklight, osPriorityNormal, 0, 128);
  BacklightHandle = osThreadCreate(osThread(Backlight), NULL);

  /* definition and creation of FoldingMirrors */
  osThreadDef(FoldingMirrors, ProctFoldingMirrors, osPriorityNormal, 0, 128);
  FoldingMirrorsHandle = osThreadCreate(osThread(FoldingMirrors), NULL);

  /* definition and creation of Can */
  osThreadDef(Can, ProcCan, osPriorityNormal, 0, 128);
  CanHandle = osThreadCreate(osThread(Can), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; 
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	//sFilterConfig.SlaveStartFilterBank = 14;

	if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
	Error_Handler();
	}
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_out_GPIO_Port, led_out_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, backlight_Pin|glass_down_Pin|glass_up_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, mirror_heating_Pin|open_the_door_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led_out_Pin */
  GPIO_InitStruct.Pin = led_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : backlight_Pin glass_down_Pin glass_up_Pin */
  GPIO_InitStruct.Pin = backlight_Pin|glass_down_Pin|glass_up_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : mirror_heating_Pin open_the_door_Pin */
  GPIO_InitStruct.Pin = mirror_heating_Pin|open_the_door_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : fr_glass_up_btn_Pin fr_glass_down_btn_Pin door_limit_switch_Pin door_opening_button_Pin
                           glass_up_limit_switch_Pin glass_down_limit_switch_Pin */
  GPIO_InitStruct.Pin = fr_glass_up_btn_Pin|fr_glass_down_btn_Pin|door_limit_switch_Pin|door_opening_button_Pin
                          |glass_up_limit_switch_Pin|glass_down_limit_switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void glass_stop()
{
	GPIOA->BRR = glass_up_Pin; // реле подьема стекла выкл
	GPIOA->BRR = glass_down_Pin; // реле опускания стекла выкл
	fr_glass = glass_Stop;
	RxData[0] = 255; //
	TxData[0] = 255; //
}
void glass_up()
{
	GPIOA->BSRR = glass_up_Pin; // реле подьема стекла вкл
	GPIOA->BRR = glass_down_Pin; // реле опускания стекла выкл
	fr_glass = glass_Up;
	RxData[0] = 255; //
	TxData[0] = 255; //
}
void glass_down()
{
	GPIOA->BRR = glass_up_Pin; // реле подьема стекла выкл
	GPIOA->BSRR = glass_down_Pin; // реле опускания стекла вкл
	fr_glass = glass_Down;
	RxData[0] = 255; //
	TxData[0] = 255; //
}
void glass_closer()
{
	glass_closer_on = 1;
	GPIOA->BSRR = glass_up_Pin; // реле подьема стекла вкл
	GPIOA->BRR = glass_down_Pin; // реле опускания стекла выкл
	fr_glass = glass_Up;
	RxData[0] = 255; //
	TxData[0] = 255; //
}
void glass_bloking_on()
{
	glass_blocking = 1;
	glass_stop();
	RxData[0] = 255; //
	TxData[0] = 255; //
}
void glass_bloking_off()
{
	glass_blocking = 0;
	RxData[0] = 255; //
	TxData[0] = 255; //
}
void mirror_heating_on()
{
	GPIOB->BSRR = mirror_heating_Pin;
	RxData[2] = 255; //
	TxData[2] = 255; //
}
void mirror_heating_off()
{
	GPIOB->BRR = mirror_heating_Pin;
	RxData[2] = 255; //
	TxData[2] = 255; //
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		if (adc1_flag) {
			adc1_flag = 0;
			HAL_ADCEx_InjectedStart_IT(&hadc1);
		}
    osDelay(10);
  }
  /* USER CODE END 5 */
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
		osDelay(1);
  }
  /* USER CODE END ProctDisplay */
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
		if(!glass_blocking && !Can_Rx && !glass_closer_on){ //
			if( !(GPIOB->IDR &fr_glass_up_btn_Pin) ){ // кнопка подьема стекла
				if(!fr_glass){
					fr_glass_btn_time = 0;
					glass_up();
				}
				fr_glass_btn_time++;
			}			
			else {
				if(fr_glass == glass_Up && fr_glass_btn_time>glass_btn_time) glass_stop();
			}
			
			if( !(GPIOB->IDR &fr_glass_down_btn_Pin) ) { // кнопка опускания стекла
				if(!fr_glass){
					fr_glass_btn_time = 0;
					glass_down();
				}
				fr_glass_btn_time++;
			}			
			else {
				if(fr_glass == glass_Down && fr_glass_btn_time>glass_btn_time) glass_stop();
			}	
		}

	if((GPIOA->IDR &glass_up_Pin || GPIOA->IDR &glass_down_Pin ) && current>fr_motor_current){ // отключение п току
			osDelay(200);
			if(current>fr_motor_current){
				glass_stop();
				if(glass_closer_on){
					TxHeader.StdId = can_addr_door_fl; 
					TxData[0] = glass_closer_Ok; //
					while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
					if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
					{
				//		HAL_UART_Transmit(&huart1, (uint8_t*)"ER SEND\n", 8, 100);
					}
					RxData[0] = 255; //
					TxData[0] = 255; //
					glass_closer_on = 0;
				}
				osDelay(400);
			}
		}
		osDelay(1);
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
	  osDelay(1);
  }
  /* USER CODE END ProctOutputs */
}

/* USER CODE BEGIN Header_ProctBacklight */
/**
* @brief Function implementing the Backlight thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProctBacklight */
void ProctBacklight(void const * argument)
{
  /* USER CODE BEGIN ProctBacklight */
  /* Infinite loop */
  for(;;)
  {
		if( !(GPIOB->IDR &door_limit_switch_Pin) ) { // концевик двери
			if(!(GPIOA->IDR &backlight_Pin)) GPIOA->BSRR = backlight_Pin; // выход uln2003a на подсветку
		}
		else 	if(GPIOA->IDR &backlight_Pin) GPIOA->BRR = backlight_Pin; // выход uln2003a на подсветку
		
		if( !(GPIOB->IDR &door_opening_button_Pin) ) { // кнопка открытия двери
			GPIOA->BSRR = open_the_door_Pin; // выход uln2003a на мотор открытия двери
			osDelay(1000);
			GPIOA->BRR = open_the_door_Pin; // выход uln2003a на мотор открытия двери
		}
		
    osDelay(100);
  }
  /* USER CODE END ProctBacklight */
}

/* USER CODE BEGIN Header_ProctFoldingMirrors */
/**
* @brief Function implementing the FoldingMirrors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProctFoldingMirrors */
void ProctFoldingMirrors(void const * argument)
{
  /* USER CODE BEGIN ProctFoldingMirrors */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ProctFoldingMirrors */
}

/* USER CODE BEGIN Header_ProcCan */
/**
* @brief Function implementing the Can thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProcCan */
void ProcCan(void const * argument)
{
  /* USER CODE BEGIN ProcCan */
  /* Infinite loop */
  for(;;)
  {
		if(Can_Rx){
			switch (RxData[0])
			{
				case 20: glass_stop();
					break;
				case 21: glass_up();
					break;
				case 22: glass_down();
					break;
				case glass_closer_On: glass_closer();
					break;
				case glass_blocking_On: glass_bloking_on();
					break;
				case glass_blocking_Off: glass_bloking_off();
					break;
				default: 
					break;
			}
			switch (RxData[2])
			{
				case 20: mirror_heating_off();
					break;
				case 21: mirror_heating_on();
					break;
				default:
					break;
			}
			Can_Rx = 0;
		}
    osDelay(1);
  }
  /* USER CODE END ProcCan */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
