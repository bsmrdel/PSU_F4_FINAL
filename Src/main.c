/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "arm_math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"
#include <stm32f4xx_hal.h>
#include <stm32_hal_legacy.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
arm_pid_instance_f32 PID;     //ARM PID instance float 32b

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VOLT_DIV_FACTOR		0.0595 	// assuming R1 = 16.4k and R2 = 1k (3V / 0.595 = 50.4)
// this is still dependent on resistor tolerances
#define CURR_DIV_FACTOR 	0.5		// CSA gain is 0.5V/A, ref to 3V...
// 0A -> 3V, 1A -> 2.5V, 2A -> 2V, 6A -> 0V


#define VOLT_OFFSET			-0.1		// CALIBRATE
#define CURR_OFFSET			0		// CALIBRATE
#define CURR_REF			3		// reference voltage for CSA, CALIBRATE THIS
#define cc_hysterisis       0.01	// to prevent quick jumps b/w CC and CV modes
#define N					1000		// moving avg approx uses 100 past samples

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

int error_voltage = 0;   //rename and write conditions for errors
int error_current = 0;
int error_temp = 0;
int OutputCounter = 0;

int OutputOn = 0;
int OutputOff = 0;

int User_Voltage_limit = 0;  //set from encoder
int VDecimal = 1;
int VDecimalCounter = 0;
int VDecimalOn = 0;
int VDecimalLast = 0;
int VaState = 0;
int VbState = 0;
int VaLastState = 0;
int VbLastState = 0;
int vd1 = 0;
int vd2 = 0;
int vd3 = 0;
int vd4 = 1;

int User_Current_limit = 0;  //set from encoder
int CDecimal = 1;
int CDecimalCounter = 0;
int CDecimalOn = 0;
int CDecimalLast = 0;
int CaState = 0;
int CbState = 0;
int CaLastState = 0;
int CbLastState = 0;
int cd1 = 0;
int cd2 = 0;
int cd3 = 0;
int cd4 = 1;

int Watts = 0;  	//Power being displayed to user, calculated from output V and I
int LastWatts = 0;
int farh_avg_int = 0;
int Lastfarh = 0;
int maxreset = 0;
int ResetOn  = 0;
int ResetOff  = 0;

char* data = "Hello World\n\r";
char tx[25];
int count = 0;

//RGB LED
int i = 0;
int up = 0;

int raw_tempsense_value = 0; 	//12b value from adc for TempSense
float farh = 0;
int max_trans_current = 0;

int raw_voltage = 0;            //12b value from adc for vsense
int raw_current = 0;            //12b value from adc for isense
float pwm_val = 0;				//PWM value for duty cycle adjustment to gate driver
float pwm_val_avg = 0;
float v_sense = 0;				//buck output voltage sense
float i_sense = 0;				//buck output current sense
float rload = 0;            	//buck calculated load from v_sense & i_sense
float pid_error = 0;			//difference from target v or i and sensed v or i
int user_en = 1;            	//output enable flag
float i_lim = 0;        		//user selected current limit @elliott  made changes and uses actuall set limit
float v_lim = 0;           		//user selected voltage limit @elliott made changes

float v_sense_avg = 0;			//moving average val of v_sense
float i_sense_avg = 0;			//moving average val of i_sense
float farh_avg = 0;
float current = 0;              					//0-3V conversion for voltage
int v_sense_avg_int = 0;
int i_sense_avg_int = 0;
int cvcc_flag = 0;				//constant voltage = 1, constant current = 0 (modes of operation)

float PID_Kp = 10;             	//proportional gain
float PID_Ki = 0.0006;         	//integral gain
float PID_Kd = 300;             //derivative gain


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

void OutputEnable(int error_voltage, int error_current, int error_temp);
void getVoltage_limit(void);
void getCurrent_limit(void);



static float approxMovingAvg(float avg, float new_sample);
void senseADC(void);
void getV(void);
void getI(void);
void getPower(void);
void getTemp(void);
void FanPWM(void);
void getMode(void);
void max_trans(void);
void PIDsetBuckPWM();

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){	//Print Function

	if (count > 15){
		count = 0;
	}

	switch(count)
	{
	case 0:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "vd1.val=%d%c%c%c",vd1,255,255,255));
		break;
	case 1:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "vd2.val=%d%c%c%c",vd2,255,255,255));
		break;
	case 2:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "vd3.val=%d%c%c%c",vd3,255,255,255));
		break;
	case 3:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "vd4.val=%d%c%c%c",vd4,255,255,255));
		break;
	case 4:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "VoltageL.val=%d%c%c%c",User_Voltage_limit,255,255,255));
		break;
	case 5:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "VoltageO.val=%d%c%c%c",v_sense_avg_int,255,255,255));
		break;
	case 6:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "cd1.val=%d%c%c%c",cd1,255,255,255));
		break;
	case 7:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "cd2.val=%d%c%c%c",cd2,255,255,255));
		break;
	case 8:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "cd3.val=%d%c%c%c",cd3,255,255,255));
		break;
	case 9:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "cd4.val=%d%c%c%c",cd4,255,255,255));
		break;
	case 10:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "CurrentL.val=%d%c%c%c",User_Current_limit,255,255,255));
		break;
	case 11:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "CurrentO.val=%d%c%c%c",i_sense_avg_int,255,255,255));
		break;
	case 12:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "Trans.val=%d%c%c%c",max_trans_current,255,255,255));
		break;
	case 13:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "TransON.val=%d%c%c%c",maxreset,255,255,255));
		break;
	case 14:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "Watts.val=%d%c%c%c",Watts,255,255,255));
		break;
	case 15:
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "Temp.val=%d%c%c%c",farh_avg_int,255,255,255));
		break;
		//	case 16:
		//		if(error_temp == 1){
		//			HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "Temp.pco=%d%c%c%c",63488,255,255,255));	//turns temp number red on screen
		//		}
		//		break;
		//	case 17:
		//		if(error_temp == 1){
		//			HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "temptxt.pco=%d%c%c%c",63488,255,255,255));//turns temp text red on screen
		//		}
		//		break;
		//	case 18:
		//		if(error_temp == 1){
		//			HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx, sprintf(tx, "ftxt.pco=%d%c%c%c",63488,255,255,255));	//turns F text red on screen
		//		}
		//		break;
	}
	count++;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){	//RGB LED Interrupt

	if( htim -> Instance ==TIM4){	//Fade red and blue
		if(i<2){
			up = 1;
		}
		if(i>100){
			up = 0;
		}
		if(up == 1){
			i++;
		}
		if(up == 0){
			i--;
		}
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, i);	//
	}
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

	PID.Kp = PID_Kp;
	PID.Ki = PID_Ki;
	PID.Kd = PID_Kd;

	arm_pid_init_f32(&PID, 1);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 		//Start PWM_HI for Buck
	//HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); 	//Start PWM_LOW for Buck
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); 		//Start PWM for FAN
	//	HAL_TIM_Base_Start_IT(&htim3);           	//Start Interrupt Timer for 3
	HAL_UART_Transmit_IT(&huart3, (uint8_t *)data, 16);	//Begin Printing

	//RGB LED strip
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim4);           //Start Interrupt Timer for 4

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		//User Interface
		OutputEnable(error_voltage,error_current,error_temp);	//Output enable function, error parameters can be used to send error signal to disable output
		getVoltage_limit();
		getCurrent_limit();

		v_lim = User_Voltage_limit/100.0;
		i_lim = User_Current_limit/100.0;

		//		//temporary limit hardcoding for PID debug
		//		v_lim = 3; 	//V
		//		i_lim = 0.2; 	//A

		//user end
		senseADC();
		getV();
		getI();
		getPower();
		rload = v_sense_avg / i_sense_avg;

		getMode();			//1 = Const V, 0 = Const C mode
		PIDsetBuckPWM();	//set new PWM for buck using PID loop

		getTemp();
		max_trans();  		//Checks and displays max transient current when OE is pressed.
		FanPWM();

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 336-1;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 980-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 25;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 250-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 20-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 100-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Output_enable_LED_Pin|CC_LED_Pin|CV_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE6 Overvoltage_SFTY_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_6|Overvoltage_SFTY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Output_enable_LED_Pin CC_LED_Pin CV_LED_Pin */
  GPIO_InitStruct.Pin = Output_enable_LED_Pin|CC_LED_Pin|CV_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Current_Decimal_OFF_Pin Current_Decimal_ON_Pin Output_Enable_OFF_Pin Output_Enable_ON_Pin 
                           Voltage_Decimal_OFF_Pin Voltage_Decimal_ON_Pin Max_transient_Reset_OFF_Pin Max_transient_Reset_ON_Pin 
                           Current_Encoder_B_Pin Current_Encoder_A_Pin */
  GPIO_InitStruct.Pin = Current_Decimal_OFF_Pin|Current_Decimal_ON_Pin|Output_Enable_OFF_Pin|Output_Enable_ON_Pin 
                          |Voltage_Decimal_OFF_Pin|Voltage_Decimal_ON_Pin|Max_transient_Reset_OFF_Pin|Max_transient_Reset_ON_Pin 
                          |Current_Encoder_B_Pin|Current_Encoder_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Voltage_Encoder_A_Pin Voltage_Encoder_B_Pin */
  GPIO_InitStruct.Pin = Voltage_Encoder_A_Pin|Voltage_Encoder_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void OutputEnable(int error_voltage, int error_current, int error_temp){

	int Overvoltage = 0;

	OutputOn = HAL_GPIO_ReadPin(Output_Enable_ON_GPIO_Port,Output_Enable_ON_Pin);
	OutputOff = HAL_GPIO_ReadPin(Output_Enable_OFF_GPIO_Port,Output_Enable_OFF_Pin);
	Overvoltage = HAL_GPIO_ReadPin(Overvoltage_SFTY_GPIO_Port,Overvoltage_SFTY_Pin);

	if (Overvoltage > 0){
		error_voltage = 1;
	}

	if(error_voltage == 1 || error_current == 1 || error_temp == 1){		//if an error has occurred
		OutputOn = 0;	//make output turn off
		OutputOff = 1;	//make output turn off
	}

	if (OutputOn == 0 && OutputOff != 0 && OutputCounter == 0){		//if error or button has been pressed once
		OutputCounter ++;
		HAL_GPIO_WritePin(Output_enable_LED_GPIO_Port,Output_enable_LED_Pin , GPIO_PIN_RESET); //turn off output enable led
	}
	else if(OutputOn != 0 && OutputOff == 0 && OutputCounter == 1){	//if no error and button has been pressed twice
		OutputCounter = 0;
		HAL_GPIO_WritePin(Output_enable_LED_GPIO_Port,Output_enable_LED_Pin , GPIO_PIN_SET);  //turn on output enable led
	}
}

void getVoltage_limit(void){
	VDecimalOn = HAL_GPIO_ReadPin(Voltage_Decimal_ON_GPIO_Port,Voltage_Decimal_ON_Pin);		//read input from PA0 Reads the "current" state of the button
	//VDecimalLast = HAL_GPIO_ReadPin(Voltage_Decimal_OFF_GPIO_Port,Voltage_Decimal_OFF_Pin);	//read input from PA1 Reads the "current" state of the button

	if (VDecimalOn != 0 && VDecimalOn != VDecimalLast && VDecimalCounter == 0){    //if button has been pressed once
		VDecimal = 1;		//start incrementing voltage by 0
		VDecimalCounter ++;	//increment counter
		vd1 = 0;			//set decimal place indicator
		vd2 = 0;			//set decimal place indicator
		vd3 = 0;			//set decimal place indicator
		vd4 = 1;			//set decimal place indicator
	}
	else if(VDecimalOn != 0 && VDecimalOn != VDecimalLast && VDecimalCounter == 1){	//if button has been pressed twice
		VDecimal = 10;		//start incrementing voltage by 10
		VDecimalCounter ++;	//increment counter
		vd1 = 0;			//set decimal place indicator
		vd2 = 0;			//set decimal place indicator
		vd3 = 1;			//set decimal place indicator
		vd4 = 0;			//set decimal place indicator
	}
	else if(VDecimalOn != 0 && VDecimalOn != VDecimalLast && VDecimalCounter == 2){	//if button has been pressed three times
		VDecimal = 100;		//start incrementing voltage by 100
		VDecimalCounter ++;	//increment counter
		vd1 = 0;			//set decimal place indicator
		vd2 = 1;			//set decimal place indicator
		vd3 = 0;			//set decimal place indicator
		vd4 = 0;			//set decimal place indicator
	}
	else if(VDecimalOn != 0 && VDecimalOn != VDecimalLast && VDecimalCounter == 3){	//if button has been pressed four times
		VDecimal = 1000;	//start incrementing voltage by 1000
		VDecimalCounter = 0;//reset counter
		vd1 = 1;			//set decimal place indicator
		vd2 = 0;			//set decimal place indicator
		vd3 = 0;			//set decimal place indicator
		vd4 = 0;			//set decimal place indicator
	}
	VDecimalLast = VDecimalOn;

	//VOLTAGE COUNTER
	VaState = HAL_GPIO_ReadPin(Voltage_Encoder_A_GPIO_Port,Voltage_Encoder_A_Pin);	//read input from PC6 // Reads the "current" state of the outputA
	VbState = HAL_GPIO_ReadPin(Voltage_Encoder_B_GPIO_Port,Voltage_Encoder_B_Pin);	//read input from PC7 // Reads the "current" state of the outputB

	if (VaState != VaLastState && VaState > VbState){		//if the previous and the current state of the outputA are different, that means a Pulse has occurred and if the outputB state is different to the outputA state, that means the encoder is rotating clockwise
		User_Voltage_limit = User_Voltage_limit + VDecimal;	//increment voltage
		if (User_Voltage_limit > 3200) {       				//our power supply cannot go over 32 V
			User_Voltage_limit = 3200;         				//don't allow voltage setting above 32 V
		}
	}
	else if (VbState != VbLastState && VbState > VaState){	//if the previous and the current state of the outputA are different, that means a Pulse has occurred and //if the outputB state is different to the outputA state, that means the encoder is rotating clockwise
		User_Voltage_limit = User_Voltage_limit - VDecimal;	//increment voltage
		if (User_Voltage_limit < 0) {       				//our power supply cannot go over 32 V
			User_Voltage_limit = 0;         				//don't allow voltage setting above 32 V
		}
	}
	VaLastState = VaState;          						//updates the previous state of the outputA with the current state
	VbLastState = VbState;									//updates the previous state of the outputB with the current state
}

void getCurrent_limit(void){

	CDecimalOn = HAL_GPIO_ReadPin(Current_Decimal_ON_GPIO_Port,Current_Decimal_ON_Pin);		//read input from PA2 Reads the "current" state of the button
	//CDecimalLast = HAL_GPIO_ReadPin(Current_Decimal_OFF_GPIO_Port,Current_Decimal_OFF_Pin);	//read input from PA3 Reads the "current" state of the button

	if (CDecimalOn != 0 && CDecimalOn != CDecimalLast && CDecimalCounter == 0){	//if button has been pressed once
		CDecimal = 1;		//start incrementing voltage by 0
		CDecimalCounter ++;	//increment counter
		cd1 = 0;			//set decimal place indicator
		cd2 = 0;			//set decimal place indicator
		cd3 = 0;			//set decimal place indicator
		cd4 = 1;			//set decimal place indicator
	}
	else if(CDecimalOn != 0 && CDecimalOn != CDecimalLast && CDecimalCounter == 1){	//if button has been pressed twice
		CDecimal = 10;		//start incrementing voltage by 10
		CDecimalCounter ++;	//increment counter
		cd1 = 0;			//set decimal place indicator
		cd2 = 0;			//set decimal place indicator
		cd3 = 1;			//set decimal place indicator
		cd4 = 0;			//set decimal place indicator
	}
	else if(CDecimalOn != 0 && CDecimalOn != CDecimalLast && CDecimalCounter == 2){	//if button has been pressed three times
		CDecimal = 100;		//start incrementing voltage by 100
		CDecimalCounter ++;	//increment counter
		cd1 = 0;			//set decimal place indicator
		cd2 = 1;			//set decimal place indicator
		cd3 = 0;			//set decimal place indicator
		cd4 = 0;			//set decimal place indicator
	}
	else if(CDecimalOn != 0 && CDecimalOn != CDecimalLast && CDecimalCounter == 3){	//if button has been pressed four times
		CDecimal = 1000;	//start incrementing voltage by 1000
		CDecimalCounter = 0;//reset counter
		cd1 = 1;			//set decimal place indicator
		cd2 = 0;			//set decimal place indicator
		cd3 = 0;			//set decimal place indicator
		cd4 = 0;			//set decimal place indicator
	}
	CDecimalLast = CDecimalOn;

	//CURRENT COUNTER
	CaState = HAL_GPIO_ReadPin(Current_Encoder_A_GPIO_Port,Current_Encoder_A_Pin);	//read input from PA4 // Reads the "current" state of the outputA
	CbState = HAL_GPIO_ReadPin(Current_Encoder_B_GPIO_Port,Current_Encoder_B_Pin);	//read input from PA5// Reads the "current" state of the outputB

	if (CaState != CaLastState && CaState > CbState){		//if the previous and the current state of the outputA are different, that means a Pulse has occurred and if the outputB state is different to the outputA state, that means the encoder is rotating clockwise
		User_Current_limit = User_Current_limit + CDecimal;	//increment voltage
		if (User_Current_limit > 300) {       				//our power supply cannot go over 32 V
			User_Current_limit = 300;         				//don't allow voltage setting above 32 V
		}
	}
	else if (CbState != CbLastState && CbState > CaState){	//if the previous and the current state of the outputA are different, that means a Pulse has occurred and //if the outputB state is different to the outputA state, that means the encoder is rotating clockwise
		User_Current_limit = User_Current_limit - CDecimal;	//increment voltage
		if (User_Current_limit < 0) {       				//our power supply cannot go over 32 V
			User_Current_limit = 0;         				//don't allow voltage setting above 32 V
		}
	}
	CaLastState = CaState;          						//updates the previous state of the outputA with the current state
	CbLastState = CbState;									//updates the previous state of the outputB with the current state
}

void senseADC (void){

	HAL_ADC_Start(&hadc1);                             //start ADC
	HAL_ADC_PollForConversion(&hadc1, 10);             //poll until complete
	raw_voltage = HAL_ADC_GetValue(&hadc1);            //collect raw voltage
	HAL_ADC_Stop(&hadc1);                              //stop ADC


	HAL_ADC_Start(&hadc2);                             //start ADC
	HAL_ADC_PollForConversion(&hadc2, 10);             //poll until complete
	raw_current = HAL_ADC_GetValue(&hadc2) ;            //collect raw current
	HAL_ADC_Stop(&hadc2);


	HAL_ADC_Start(&hadc3);                             //start ADC
	HAL_ADC_PollForConversion(&hadc3, 10);             //poll until complete
	raw_tempsense_value = HAL_ADC_GetValue(&hadc3);    //collect raw tempval
	HAL_ADC_Stop(&hadc3);


	return;
}

void getV (void){

	float voltage = 0;             						//0-3V conversion for voltage
	float percent_voltage = 0;      					//%V from 0-3

	percent_voltage = ((float) raw_voltage) / 4092;
	voltage = percent_voltage * 3;						//0-3V ADC signal
	v_sense = voltage / VOLT_DIV_FACTOR;				//0-50V value
	v_sense_avg = approxMovingAvg(v_sense_avg, v_sense);//take average
	v_sense_avg_int = (int)(v_sense_avg*100);			//make variable compatible with screen

	if (v_sense_avg_int > 3200){						//if voltage is above safe value, return error
		error_voltage = 1;								//set error as 1 to turn off outputs
	}
	else {
		error_voltage = 0;								//set error to 0 if nothing is wrong
	}

	return;
}

void getI (void){

	float percent_current = 0;      					//%V from 0-3

	percent_current = ((float) raw_current) / 4092;		//raw percent voltage 0-3
	current =  CURR_REF - 2*(percent_current * 3);					//0-3V ADC signal
	i_sense = (current / CURR_DIV_FACTOR) + CURR_OFFSET;//0-3A value
	i_sense_avg = approxMovingAvg(i_sense_avg, i_sense);//take average
	i_sense_avg_int = (int)(i_sense_avg*100);			//make variable compatible with screen

	if (i_sense_avg_int > 300){							//if voltage is above safe value, return error
		error_current = 1;								//set error as 1 to turn off outputs
	}
	else {
		error_current = 0;								//set error to 0 if nothing is wrong
	}

	return;
}

void getPower (void){

	Watts = v_sense_avg_int * i_sense_avg_int / 100;	//calculate power
	return;
}

void getTemp (void){

	farh = ((raw_tempsense_value/4096.0)*3000.0); 	//calculate the farenheit using 5V
	farh_avg = approxMovingAvg(farh_avg,farh);		//finds average of temperature
	farh_avg_int = (int)(farh_avg);					//make variable compatible with screen

	if((farh_avg_int >= 1000 || farh_avg_int < 0) && error_temp == 0){	//if temperature rises above 100 F, alert user.
		error_temp = 1;													//an error has occurred
	}
	else if(farh_avg_int < 1000 && farh_avg_int > 0 && error_temp == 1){//if nothing is wrong and something was wrong before
		error_temp = 0;													//an error has not occurred
		//					printf("Temp.pco=%d%c%c%c",65535,255,255,255);					//turns temp number white on screen Temp.pco=0ÿÿÿ
		//					printf("temptxt.pco=%d%c%c%c",65535,255,255,255);				//turns temp text white on screen tempxt.pco=0ÿÿÿ
		//					printf("ftxt.pco=%d%c%c%c",65535,255,255,255);					//turns F text white on screen ftxt.pco=0ÿÿÿ
		return;
	}
}

void FanPWM (void){

	float fan_duty=0;

	if(farh_avg_int>800){
		fan_duty = 80;
	}
	else{
		fan_duty = 30;
	}

	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,fan_duty);

}

static float approxMovingAvg(float avg, float newsample)
{
	avg -= avg / N;
	avg += newsample / N;

	return avg;
}

void getMode(void){

	if(cvcc_flag == 0){   //check if you were in cc mode
		if(i_sense_avg >=(i_lim - cc_hysterisis)){ //checking lower range
			//cc mode
			cvcc_flag = 0;
		}
		else
			cvcc_flag = 1;		//otherwise assume cv mode
	}
	else{
		if(i_sense_avg >=i_lim){ //checking lower range
			//cc mode
			cvcc_flag = 0;
		}
		else
			cvcc_flag = 1;		//otherwise assume cv mode

	}

	// turn on top STM32F0disc LED for CV, bottom for CC mode for debug
	if(cvcc_flag == 1){
		//breadboard LEDS
		HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CV_LED_GPIO_Port, CV_LED_Pin, GPIO_PIN_SET);
	}
	else
	{

		// breadboard LEDS
		HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(CV_LED_GPIO_Port, CV_LED_Pin, GPIO_PIN_RESET);
	}
	return;
}

void max_trans(void){

	ResetOn  = HAL_GPIO_ReadPin(Max_transient_Reset_ON_GPIO_Port,Max_transient_Reset_ON_Pin);	//read input from PB0 Reads the "current" state of the button
	ResetOff = HAL_GPIO_ReadPin(Max_transient_Reset_OFF_GPIO_Port,Max_transient_Reset_OFF_Pin);	//read input from PB1 Reads the "current" state of the button

	if (ResetOn > ResetOff){						//if button has been pressed once
		if (i_sense_avg_int > max_trans_current){	//if the previous and the current state of the outputA are different, that means a Pulse has occurred
			max_trans_current = i_sense_avg_int;	//updates the previous state of the average current sense with the current state
			maxreset = 1;							//indicator on
		}
	}
	else if(ResetOn < ResetOff && maxreset == 1){	//if button has been pressed twice
		max_trans_current = 0;						//Reset max transient current
		maxreset = 0;								//indicator off
	}
}

void PIDsetBuckPWM(void){

	if(cvcc_flag == 1){	//if in CV mode
		pid_error = v_lim - v_sense_avg;
	}else{			//in CC mode
		pid_error = i_lim - i_sense_avg;
	}

	if(OutputOn == 0){
		pid_error = 0; //do not accumulate error if output is disabled via OE button
	}

	pwm_val = arm_pid_f32(&PID, pid_error);
	pwm_val_avg = approxMovingAvg(pwm_val_avg, pwm_val);
	//FOR TESTING
	//pwm_val = i_sense_avg * 30;
	/* 10% = 35
	 * 20% = 70
	 * 30% = 105
	 * 40% = 140
	 * 50% = 175
	 *
	 * 95% = 330
	 * 100% = 350
	 */

	//capture max or min PWM to prevent out of range duty cycle (0-95%)
	if(pwm_val_avg > 105) //330 is max for 95% duty cycle on PWM_HI
		pwm_val_avg = 105; //28%

	if(pwm_val_avg < 0)
		pwm_val_avg = 0;

	//hardcode to test 12V line
	//pwm_val_avg	= 0;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_val_avg);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
