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
#include "fonts.h"
#include "ssd1306.h"
#include "string.h"
#include "DFPLAYER_MINI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t emergy = 0;

#define starter_time 150 // Время вращения стартера. Стоит полторы секунды
#define tach_run 150 // скорость стартера по stm при умножении на 30 по ECU 266
#define voltage_engine_run 12.5 // наприжение при котором считается что двигатель запущен
uint8_t engine_startup_attempts = 5; // Количество попыток запуска двигателя
#define acc_off_time_d 20 //время в секундах
//#define gear_led 2500

#define engine_temperature_stop 0 // 0 отключено
#define inside_temperature_stop 22

#define can_addr_door_fl 0x0001
#define can_addr_door_fr 0x0002
#define can_addr_climate 0x0006

#define fl_glass_closer_On 13
#define glass_closer_ok 14

#define fl_glass_blocking_On 15

#define fl_mirror_folding_On 11
#define fl_mirror_folding_Off 10
#define fl_mirror_heating_On 11
#define fl_mirror_heating_Off 10

#define climate_On 61
#define climate_Off 60

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId DisplayHandle;
osThreadId AnalogReadHandle;
osThreadId DigitalReadHandle;
osThreadId ButtonReadHandle;
osThreadId OutputsHandle;
osThreadId EngineStartHandle;
osThreadId EngineStopHandle;
osThreadId IdleTaskHandle;
osThreadId EngineRestartHandle;
osThreadId CanHandle;
osThreadId ProctImmoBTaskHandle;
/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = {255, 255, 255, 255, 255, 255};
uint8_t RxData[8] = {255, 255, 255, 255, 255, 255};
uint32_t TxMailbox = 0;
bool Can_Rx = 0;

bool glass_closer_all_ok = 0;
//char pcWriteBuffer [1024];
//volatile int freemem;

bool hand_brake_flag = 0, neutral_flag = 0, clutch_flag = 0, brake_flag = 0;
bool oil_lamp_flag = 0, batt_lamp_flag = 0, engine_lamp_flag = 0, immo_flag = 0, fuel_pump_flag = 0;

bool autostart_flag = 0, autostart_acc_flag = 0;
bool engine_starting_flag = 0, engine_run_flag = 0, engine_stoping_flag = 0, engine_run_ok_flag = 0;
uint16_t starter_run = 0;
uint8_t starting_error = 0;

bool state = 0;
bool immo_false = 0;
uint16_t immo_blink_time = 0, immo_blink_time_old = 0, while_time = 5000;

bool adc1_flag = 0, adc2_flag = 0;
float voltage_value = 0, voltage = 0, voltage_start = 15, voltage_run = voltage_engine_run;

float temperature_value = 0, temperature = 0;
uint8_t warming_up = engine_temperature_stop;

float fuel_level_value = 0, fuel_level = 0;

uint8_t disp = 1;
uint8_t brightness = 254;
char lcd1602_tx_buffer[40]; //глобальный буфер данных. В него записывается текст.
extern const uint8_t logo[1024];

extern bool tim_flag;
uint16_t count_tach = 0;
uint16_t count_tach_over = 0;
uint32_t tach = 0, tach_start = 0;

uint16_t count_speed = 0;
uint16_t count_speed_over = 0;
uint32_t speed = 0;

extern uint8_t status_old;

uint8_t btn_time = 20, button_press = 0;
uint8_t encoder = 1, encoder_old = 0;

uint16_t acc_off_time = acc_off_time_d;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C2_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
void ProctDefaultTask(void const * argument);
void ProctDisplay(void const * argument);
void ProctAnalogRead(void const * argument);
void ProctDigitalRead(void const * argument);
void ProctButtonRead(void const * argument);
void ProctOutputs(void const * argument);
void ProctEngineStart(void const * argument);
void ProctEngineStop(void const * argument);
void ProctIdle(void const * argument);
void ProctEngineRestart(void const * argument);
void ProctCan(void const * argument);
void ProctImmoB(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if(hadc->Instance == ADC1){
		voltage_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
		voltage = (voltage_value*14.90f)/4095;
		adc1_flag = 1;
	}
	if(hadc->Instance == ADC2){
		temperature_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
		temperature = 130 - (temperature_value*130.0f)/(voltage_value); 

		fuel_level_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
		fuel_level = 55 - (fuel_level_value*50.0f)/(voltage_value/4);
		adc2_flag = 1;
		// почти полный бак значение 128 по идее полный бак будет 0
		// 988 стреока на ноле. почти пустой
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
		if(RxHeader.StdId == 0x0006) //   
    {
			Can_Rx = 1;
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);	
//			snprintf(trans_str, 128, "ID %04lX %d\n", RxHeader.StdId, RxData[0]);
//  	  HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 100);
    }
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t er = HAL_CAN_GetError(hcan);
//    sprintf(trans_str,"ER CAN %lu %08lX", er, er);
//    HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 100);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_SR_UIF);
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_SR_UIF);
	__HAL_TIM_CLEAR_FLAG(&htim3, TIM_SR_UIF);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);
	HAL_ADCEx_InjectedStart_IT(&hadc2);

	TxHeader.StdId = 0x0009;
	TxHeader.ExtId = 0;
	TxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
	TxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = 0;

	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);

	GPIOA->BRR = starter_out_Pin;
	GPIOA->BRR = immo_out_Pin;
	GPIOA->BRR = acc_out_Pin;

	SSD1306_Init();
	HAL_Delay(1000);
	
//	DF_Init(15);
//	osDelay(200);
//  DF_PlayFolderTrack(1,1);

//  DF_PlayFromStart();

	SSD1306_DrawBitmap(0,0, logo,128, 64, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(3000);
	
	if (!(GPIOA->IDR & encoder_scl_Pin) && (!(GPIOB->IDR & encoder_sda_Pin))) {
		status_old = 0x00;
	} else if ((GPIOA->IDR & encoder_scl_Pin) && (!(GPIOB->IDR & encoder_sda_Pin))) {
		status_old = 0x10;
	} else if ((GPIOA->IDR & encoder_scl_Pin) && (GPIOB->IDR & encoder_sda_Pin)) {
		status_old = 0x11;
	} else if (!(GPIOA->IDR & encoder_scl_Pin) && (GPIOB->IDR & encoder_sda_Pin)) {
		status_old = 0x01;
	}

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

  /* definition and creation of EngineStart */
  osThreadDef(EngineStart, ProctEngineStart, osPriorityNormal, 0, 128);
  EngineStartHandle = osThreadCreate(osThread(EngineStart), NULL);

  /* definition and creation of EngineStop */
  osThreadDef(EngineStop, ProctEngineStop, osPriorityNormal, 0, 128);
  EngineStopHandle = osThreadCreate(osThread(EngineStop), NULL);

  /* definition and creation of IdleTask */
  osThreadDef(IdleTask, ProctIdle, osPriorityNormal, 0, 256);
  IdleTaskHandle = osThreadCreate(osThread(IdleTask), NULL);

  /* definition and creation of EngineRestart */
  osThreadDef(EngineRestart, ProctEngineRestart, osPriorityNormal, 0, 128);
  EngineRestartHandle = osThreadCreate(osThread(EngineRestart), NULL);

  /* definition and creation of Can */
  osThreadDef(Can, ProctCan, osPriorityNormal, 0, 128);
  CanHandle = osThreadCreate(osThread(Can), NULL);

  /* definition and creation of ProctImmoBTask */
  osThreadDef(ProctImmoBTask, ProctImmoB, osPriorityNormal, 0, 128);
  ProctImmoBTaskHandle = osThreadCreate(osThread(ProctImmoBTask), NULL);

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
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
//   sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  /* USER CODE END ADC2_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_out_GPIO_Port, led_out_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(starter_rele_out_GPIO_Port, starter_rele_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, starter_out_Pin|ignition_out_Pin|immo_out_Pin|acc_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led_out_Pin */
  GPIO_InitStruct.Pin = led_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : starter_rele_out_Pin */
  GPIO_InitStruct.Pin = starter_rele_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(starter_rele_out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : starter_out_Pin ignition_out_Pin immo_out_Pin acc_out_Pin */
  GPIO_InitStruct.Pin = starter_out_Pin|ignition_out_Pin|immo_out_Pin|acc_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : brake_in_Pin engine_lamp_in_Pin */
  GPIO_InitStruct.Pin = brake_in_Pin|engine_lamp_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : hand_brake_in_Pin neutral_in_Pin oil_lamp_in_Pin batt_lamp_in_Pin
                           clutch_in_Pin autostart_in_Pin */
  GPIO_InitStruct.Pin = hand_brake_in_Pin|neutral_in_Pin|oil_lamp_in_Pin|batt_lamp_in_Pin
                          |clutch_in_Pin|autostart_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : immo_lamp_in_Pin */
  GPIO_InitStruct.Pin = immo_lamp_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(immo_lamp_in_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : fuel_pump_in_Pin */
  GPIO_InitStruct.Pin = fuel_pump_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(fuel_pump_in_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : start_button_in_Pin */
  GPIO_InitStruct.Pin = start_button_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(start_button_in_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : encoder_scl_Pin */
  GPIO_InitStruct.Pin = encoder_scl_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(encoder_scl_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : encoder_sda_Pin */
  GPIO_InitStruct.Pin = encoder_sda_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(encoder_sda_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ProctDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ProctDefaultTask */
void ProctDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		if(emergy){
			SSD1306_Init();
			SSD1306_GotoXY (0,0);
			sprintf(lcd1602_tx_buffer, "EMERGY %1d ", emergy); 
			SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
			
			SSD1306_GotoXY (0,18);
			sprintf(lcd1602_tx_buffer, "%4.1f V ", voltage); 
			SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
			SSD1306_GotoXY (77,18);
			sprintf(lcd1602_tx_buffer, "%4.1f V", voltage_start); 
			SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
			
			SSD1306_UpdateScreen();
			osDelay(2000);
			
			GPIOA->BSRR = acc_out_Pin;
			osDelay(2000);
			
			GPIOA->BSRR = immo_out_Pin; // Обход иммобилайзера вкл
			osDelay(2000);
			
			GPIOC->BSRR = starter_rele_out_Pin; // Обход реле стартера
			osDelay(2000);
			
			GPIOA->BSRR = ignition_out_Pin; // Зажигание вкл
		}
			
		else{
		
			if(immo_blink_time_old > 200 && immo_blink_time_old < 500 &&  !state){  // Включить аксуссуары, экран, кнопки 1705-1716 **// immo_blink_time_old > 1000 && immo_blink_time_old < 1950 &&
				SSD1306_Init();
				GPIOA->BSRR = acc_out_Pin;
				
				SSD1306_DrawBitmap(0,0, logo,128, 64, 1);
	//			for(int j = 0; j<30;j++)
	//			{
	//				SSD1306_Set_Brightness(64);
	//				SSD1306_UpdateScreen();
	//				osDelay(1000);
				
	//				SSD1306_Set_Brightness(brightness); //работает 254
	//				SSD1306_UpdateScreen();
	//				osDelay(1000);
				
	////			SSD1306_ON();
	////			SSD1306_OFF();
	//			}
				SSD1306_UpdateScreen();
				osDelay(1000);
				SSD1306_Clear();
				disp = 1;
				state = 1;

	//			vTaskResume(ButtonReadHandle);
	//			vTaskResume(DisplayHandle);		
			}
			
			if(immo_blink_time_old > 1000 && immo_blink_time_old < 2000 && state) { // Выключить аксуссуары, экран, кнопки  544-554
	//			taskENTER_CRITICAL();
				SSD1306_Init();
				TxHeader.StdId = can_addr_door_fl;
				TxData[0] = fl_glass_closer_On;
				TxData[1] = fl_mirror_folding_On;
				TxData[2] = fl_mirror_heating_Off;

				while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
				if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
				{
			//		HAL_UART_Transmit(&huart1, (uint8_t*)"ER SEND\n", 8, 100);
				}
				
	//			TxHeader.StdId = can_addr_climate;
	//			TxData[0] = climate_off;
	//			while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	//			if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	//			{
	//		//		HAL_UART_Transmit(&huart1, (uint8_t*)"ER SEND\n", 8, 100);
	//			}
	//			taskEXIT_CRITICAL();
				state = 0;
	//			while(!glass_closer_all_ok) osDelay(10);
				osDelay(6000);// еще подождать складывание зеркал

				glass_closer_all_ok = 0;
				SSD1306_Clear();

				if(state)	return; // выброс из функции			
				GPIOA->BRR = acc_out_Pin;
	//			vTaskSuspend(DisplayHandle);
	//			vTaskSuspend(ButtonReadHandle);
			}
		
			if(immo_blink_time_old > 1262 && immo_blink_time_old < 1472 && (GPIOA->IDR &ignition_out_Pin)){ // Нет иммобилайзера 1362-1372 
				immo_false = 1;
			}

			if(immo_blink_time_old > 1262 && immo_blink_time_old < 1472 && (GPIOA->IDR &ignition_out_Pin)){ // земерить режим программирования
				immo_false = 1;
			}	
			
			osDelay(10);
		}
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ProctDisplay */
/**
* @brief Function implementing the DisplayTask thread.
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
		if (state){
//			switch (disp)
//			{
//				case 0:
//					if (tim_flag) {
//							if (tach <= 100) {
//							sprintf(lcd1602_tx_buffer, "%8.3f Hz", (float) tach);
//							SSD1306_GotoXY (0,0);
//							SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
//						} else if (tach > 100 && tach < 100000) {
//							sprintf(lcd1602_tx_buffer, "%7.3f kHz", (float) tach / 1000);
//							SSD1306_GotoXY (0,0);
//							SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
//						} else if (tach >= 100000) {
//							sprintf(lcd1602_tx_buffer, "%7.3f MHz", (float) tach / 1000000);
//							SSD1306_GotoXY (0,0);
//							SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
//						}			
//		
//						if (speed <= 100) {
//							sprintf(lcd1602_tx_buffer, "%8.3f Hz", (float) speed);
//							SSD1306_GotoXY (0,18);
//							SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
//						} else if (speed > 100 && speed < 100000) {
//							sprintf(lcd1602_tx_buffer, "%7.3f kHz", (float) speed / 1000);
//							SSD1306_GotoXY (0,18);
//							SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
//						} else if (speed >= 100000) {
//							sprintf(lcd1602_tx_buffer, "%7.3f MHz", (float) speed / 1000000);
//							SSD1306_GotoXY (0,18);
//							SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
//						}
//						
//						SSD1306_GotoXY (0,36);
//						sprintf(lcd1602_tx_buffer, "fuel pump %1d ", fuel_pump_flag); 
//						SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
//					}
//				break;
//				case 1:
//					if (adc1_flag) {
//						SSD1306_GotoXY (0,0);
//						sprintf(lcd1602_tx_buffer, "%4.1f V ", voltage); 
//						SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
//						SSD1306_GotoXY (77,0);
//						sprintf(lcd1602_tx_buffer, "%4.1f V", voltage_start); 
//						SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
//					}
//					
//					if (adc2_flag) {
//						SSD1306_GotoXY (0,18);
//						sprintf(lcd1602_tx_buffer, "%3.0f C ", temperature); 
//						SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
//						SSD1306_GotoXY (66,18);
//						sprintf(lcd1602_tx_buffer, "%4.1f L ", fuel_level); 
//						SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
//					}
//					
//					if(tim_flag){
//						sprintf(lcd1602_tx_buffer, "%7.2f rpm", (float) tach );
//						SSD1306_GotoXY (0,36);
//						SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);
////						if(tach>gear_led && GPIOC->IDR &led_out_Pin) GPIOC->BRR = led_out_Pin;
////						if(tach<gear_led && !(GPIOC->IDR &led_out_Pin)) GPIOC->BSRR = led_out_Pin;
//					}
//					break;
//				case 2:
//					SSD1306_GotoXY (0,0);
//					sprintf(lcd1602_tx_buffer, "h brake   %1d", hand_brake_flag); 
//					SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);				
//					SSD1306_GotoXY (0,18);
//					sprintf(lcd1602_tx_buffer, "neutral   %1d", neutral_flag); 
//					SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);	
//					SSD1306_GotoXY (0,36);
//					sprintf(lcd1602_tx_buffer, "oil lamp  %1d", oil_lamp_flag); 
//					SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);	
//				break;
//				case 3:
//					SSD1306_GotoXY (0,0);
//					sprintf(lcd1602_tx_buffer, "batt lamp %1d", batt_lamp_flag); 
//					SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);				
//					SSD1306_GotoXY (0,18);
//					sprintf(lcd1602_tx_buffer, "engine    %1d", engine_lamp_flag); 
//					SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);	
//					SSD1306_GotoXY (0,36);
//					sprintf(lcd1602_tx_buffer, "immo lamp %1d", immo_flag); 
//					SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);	
//					break;
//				case 4:				
//					SSD1306_GotoXY (0,0);
//					sprintf(lcd1602_tx_buffer, "autostart %1d", autostart_flag); 
//					SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);	
//					SSD1306_GotoXY (0,18);
//					sprintf(lcd1602_tx_buffer, "clutch    %1d", clutch_flag); 
//					SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);	
//					SSD1306_GotoXY (0,36);
//					sprintf(lcd1602_tx_buffer, "brake     %1d", brake_flag); 
//					SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);	
//					break;
//				default:
//					break;
//			}
//			if (adc1_flag || adc2_flag || tim_flag){
//				taskENTER_CRITICAL();
//				SSD1306_UpdateScreen();
//				taskEXIT_CRITICAL();
//			}
			

		}
			if (tim_flag) {
				tim_flag = 0;
			}
			if (adc1_flag) {
				adc1_flag = 0;
				HAL_ADCEx_InjectedStart_IT(&hadc1);
			}
			if (adc2_flag) {
				adc2_flag = 0;
				HAL_ADCEx_InjectedStart_IT(&hadc2);
			}
		osDelay(50);
  }
  /* USER CODE END ProctDisplay */
}

/* USER CODE BEGIN Header_ProctAnalogRead */
/**
* @brief Function implementing the AnalogReadTask thread.
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
		if (state && encoder != encoder_old){
			disp = encoder;
			disp >= 5 ? disp = 0: disp;
			encoder = disp;
			encoder_old = encoder;
		}
//		if(state) {
//			SSD1306_Set_Brightness(brightness); //работает 254
//			SSD1306_UpdateScreen();
//		}
		osDelay(1);
  }
  /* USER CODE END ProctAnalogRead */
}

/* USER CODE BEGIN Header_ProctDigitalRead */
/**
* @brief Function implementing the DigitalReadTask thread.
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
		if(!(GPIOB->IDR &hand_brake_in_Pin)) hand_brake_flag = 1;
		else hand_brake_flag = 0;
		
		if(!(GPIOB->IDR &neutral_in_Pin )) neutral_flag = 1;
		else neutral_flag = 0;

		if(!(GPIOB->IDR &oil_lamp_in_Pin)) oil_lamp_flag = 1;
		else oil_lamp_flag = 0;

		if(!(GPIOB->IDR &batt_lamp_in_Pin)) batt_lamp_flag = 1; // разобратсья почему не работает
		else batt_lamp_flag = 0;

		if(!(GPIOA->IDR &engine_lamp_in_Pin)) engine_lamp_flag = 1;
		else engine_lamp_flag = 0;
		
		if(!(GPIOA->IDR &immo_lamp_in_Pin)) immo_flag = 1;
		else immo_flag = 0;

		if(!(GPIOA->IDR &fuel_pump_in_Pin)) fuel_pump_flag = 1;
		else fuel_pump_flag = 0;

		if(GPIOA->IDR &brake_in_Pin) brake_flag = 1;
		else brake_flag = 0;

//		if(!(GPIOB->IDR &autostart_in_Pin)) autostart_flag = 1;
//		else autostart_flag = 0;

		if(!(GPIOB->IDR &clutch_in_Pin)) clutch_flag = 1;
		else clutch_flag = 0;

		osDelay(1);
  }
  /* USER CODE END ProctDigitalRead */
}

/* USER CODE BEGIN Header_ProctButtonRead */
/**
* @brief Function implementing the ButtonReadTask thread.
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
		if(state){
			if( button_press ){ // !(GPIOA->IDR &start_button_in_Pin)

				if( (!engine_starting_flag && !engine_run_flag) ){
					engine_stoping_flag = 0;
					engine_starting_flag = 1;
				}
				else {
					if(autostart_flag) autostart_flag = 0;
						engine_starting_flag = 0; 
						engine_stoping_flag = 1;
				}
				
				while(!(GPIOA->IDR &start_button_in_Pin) || button_press ) osDelay(10);
			}
			
			if(autostart_flag){
				if(!(GPIOB->IDR &autostart_in_Pin)){
					if( (!engine_starting_flag && !engine_run_flag) ){
		//				vTaskSuspend(StartingEngineHandle);
					}
					else {
		//				vTaskSuspend(StartingEngineHandle);
						autostart_flag  = 0;
						if(engine_run_flag){
							immo_blink_time_old = 900;
						}
						engine_stoping_flag = 1;
						engine_starting_flag = 0;
		//				vTaskResume(StopingEngineHandle);
					}			
				}		
			}
		}
		else{
			if(!(GPIOB->IDR &autostart_in_Pin)){
				osDelay(2000);
				if( (!engine_starting_flag && !engine_run_flag) ){
	//				vTaskSuspend(StopingEngineHandle);
					autostart_flag  = 1;
					engine_stoping_flag = 0;
					engine_starting_flag = 1;
	//				vTaskResume(StartingEngineHandle);
				}
			}	
		}
		osDelay(50);
	}
  /* USER CODE END ProctButtonRead */
}

/* USER CODE BEGIN Header_ProctOutputs */
/**
* @brief Function implementing the OutputsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProctOutputs */
void ProctOutputs(void const * argument)
{
  /* USER CODE BEGIN ProctOutputs */
  /* Infinite loop */
  for(;;)
  { //остановка двигателя по температуре двигателя
//		if(autostart_flag && engine_run_flag && warming_up && adc2_flag && temperature>warming_up){
////			vTaskSuspend(StartingEngineHandle);
//			autostart_flag  = 0;
//			if(engine_run_flag) immo_blink_time_old = 900;
//			engine_stoping_flag = 1;
//			engine_starting_flag = 0;
////			vTaskResume(StopingEngineHandle);
//			osDelay(1000);
//		}
		if( voltage > voltage_start+1.0 && tach>tach_run && !engine_run_flag){ //  
			engine_run_flag = 1;
//			if(oil_lamp_flag){  // || batt_lamp_flag
//					GPIOA->BRR = starter_out_Pin; // Выключить стартер
//					GPIOA->BRR = immo_out_Pin; 		// Выключить иммобилайзер
//					GPIOA->BRR = ignition_out_Pin; 		// Выключить зажигаине
//			}

		}
			if( !oil_lamp_flag && tach > 800 && voltage > 13 && !batt_lamp_flag) engine_run_ok_flag = 1; //   лампа до вкл зыжигания уже ноль
			else {
				engine_run_ok_flag = 0; //глушить если нет давления масла
//				engine_starting_flag = 0;
//				engine_stoping_flag = 1;
			}
		//		if( oil_lamp_flag && tach < 250 && engine_run_flag ) engine_run_flag = 0; //&& batt_lamp_flag
		osDelay(10);
  }
  /* USER CODE END ProctOutputs */
}

/* USER CODE BEGIN Header_ProctEngineStart */
/**
* @brief Function implementing the EngineStart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProctEngineStart */
void ProctEngineStart(void const * argument)
{
  /* USER CODE BEGIN ProctEngineStart */
  /* Infinite loop */
  for(;;)
  {
		if(engine_starting_flag && !engine_run_flag){
//				if((GPIOA->IDR &ignition_out_Pin))  Заготовка под рестарт кнопкой по короткому нажатию без выключения зажиганиея
			do
				{
				if(!autostart_flag)	engine_startup_attempts = 1;
				else	if (engine_startup_attempts < 5) osDelay(3000);
				starter_run = 0;
				starting_error = 0;
				immo_false = 0;
				GPIOA->BSRR = immo_out_Pin; // Обход иммобилайзера вкл
				if(autostart_flag) osDelay(1500);
				osDelay(500);
				GPIOA->BSRR = ignition_out_Pin; // Зажигание вкл
				if(autostart_flag) osDelay(1500);			
				osDelay(500);

				voltage_start = voltage;
				if (neutral_flag && hand_brake_flag ){ // Проверка что включена нейтраль и ручник
					while (fuel_pump_flag) osDelay(10); // Ожидаем накачку топлива бензонасосом
								
					GPIOA->BSRR = starter_out_Pin; //Реле стартера вкл
					starter_run = 0;
					while (!engine_run_flag && starter_run < starter_time)
					{
						if(starter_run > 100 && tach < tach_run && engine_startup_attempts == 1 && !GPIOC->IDR &starter_rele_out_Pin)
						{
							starter_run = starter_time;
							GPIOC->BSRR = starter_rele_out_Pin; //разрешает запуск стартера без разрешения 
						}
						starter_run++;
						osDelay(10);
					}

					GPIOC->BRR = starter_rele_out_Pin; // Обход реле стартера выкл
					GPIOA->BRR = starter_out_Pin; // Реле стартера выкл
					
				  osDelay(2000);					
					if( !engine_run_flag && starter_run >= starter_time ){
						starting_error = 1;
						GPIOA->BRR = ignition_out_Pin; // Зажигание выкл
						GPIOA->BRR = immo_out_Pin; // Обход иммобилайзера выкл
					  engine_starting_flag = 0;
						osDelay(1000);
					}
					
					//			while( ) osDelay(1); //иммобилайзер не моргает

					if (autostart_flag && !starting_error) {
						immo_blink_time_old = 250;
						autostart_acc_flag = 1;
						osDelay(2000);
						
						// Включение подогрева зеркал
						TxHeader.StdId = can_addr_door_fl;
						TxData[2] = fl_mirror_heating_On;
						while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
						if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
						{
					//		HAL_UART_Transmit(&huart1, (uint8_t*)"ER SEND\n", 8, 100);
						}
						RxData[2] = 255;
						TxData[2] = 255;
					}
				}
				else {
					engine_starting_flag = 0;
					engine_stoping_flag = 1;
				}
					
					engine_startup_attempts --;
				} while (engine_startup_attempts && !engine_run_flag);
				
				if(!engine_run_flag){
					engine_starting_flag = 0;
					engine_stoping_flag = 1;
				}
				engine_startup_attempts = 5;
		}
    osDelay(1);
  }
  /* USER CODE END ProctEngineStart */
}

/* USER CODE BEGIN Header_ProctEngineStop */
/**
* @brief Function implementing the EngineStop thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProctEngineStop */
void ProctEngineStop(void const * argument)
{
  /* USER CODE BEGIN ProctEngineStop */
  /* Infinite loop */
  for(;;)
  {
		if(engine_stoping_flag){
			GPIOA->BRR = starter_out_Pin;
			GPIOA->BRR = ignition_out_Pin;
			GPIOA->BRR = immo_out_Pin;
			osDelay(3000);
			engine_run_flag = 0;
			if(autostart_flag) immo_blink_time_old = 1500;
			engine_stoping_flag = 0;
		}
    osDelay(10);
  }
  /* USER CODE END ProctEngineStop */
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
//		osDelay(2000);
//		freemem = xPortGetFreeHeapSize();
//		vTaskList (pcWriteBuffer);
		osDelay(1);
  }
  /* USER CODE END ProctIdle */
}

/* USER CODE BEGIN Header_ProctEngineRestart */
/**
* @brief Function implementing the EngineRestart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProctEngineRestart */
void ProctEngineRestart(void const * argument)
{
  /* USER CODE BEGIN ProctEngineRestart */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ProctEngineRestart */
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
		if(Can_Rx){
			Can_Rx = 0;
			if(RxData[0] != 255){
				switch (RxData[0])
				{
					case glass_closer_ok: glass_closer_all_ok = 1;
						break;			
					default: 
						break;
				}
				RxData[0] = 255;
			}
		}
	  osDelay(10);
  }
  /* USER CODE END ProctCan */
}

/* USER CODE BEGIN Header_ProctImmoB */
/**
* @brief Function implementing the ProctImmoBTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProctImmoB */
void ProctImmoB(void const * argument)
{
  /* USER CODE BEGIN ProctImmoB */
  /* Infinite loop */
  for(;;)
  {
		if(immo_flag && !engine_run_flag)
		{ 
			if(immo_blink_time != 0) { immo_blink_time_old = immo_blink_time; immo_blink_time = 0; }
//				if(!state){
//					SSD1306_GotoXY (0,0);
//					sprintf(lcd1602_tx_buffer, "Immo %4.0d", immo_blink_time_old ); 
//					SSD1306_Puts (lcd1602_tx_buffer, &Font_11x18, 1);					
//					SSD1306_UpdateScreen();
//				}
		}
		else{
			if(immo_blink_time < 3000) immo_blink_time++;
		}	
		
    osDelay(1);
  }
  /* USER CODE END ProctImmoB */
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
