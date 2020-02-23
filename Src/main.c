
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

#include "mpu6500.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static volatile int UART_READY  = 1;
static volatile int UART_SEND 	= 0;
static volatile int UART_RX 	= 0;
static volatile int I2C_READY 	= 1;
static volatile int MOTOR_READY = 1;
static volatile int TEST_START	= 0;			//FLAG UM EINEN TEST ZU STARTEN

static volatile int DATA_READY_IMU	= 1;

static volatile int TIMER_1MS_READY  = 0;
static volatile int TIMER_AXIS_FINISHED = 0;
 int i = 0;

 int o = 0;

 int a = 0;			//offset calibration y accel




uint8_t initData[2] = {MPU6500_RA_PWR_MGMT_1, 0x80 };
uint8_t ACCEL_XYZ[6] = {0};
uint8_t GYRO_Z[2] = {0};
uint8_t	_GYRO_Z[4] = {0};
uint8_t test;
uint32_t		timeres = 0;
uint8_t			timere 	= 0;

int j = 0;
/*
float actualValues[40] = {0};
float actualValues1[40] = {0};
float actualValues2[40] = {0};
*/


struct{
	uint32_t	SP;
	uint32_t	timerVal;
	uint32_t	state;
	uint32_t	last_state;

}setpointFunc;

struct
{
	uint32_t	IC_Value;
	float		faktor;
	float		period;
	float		frequency_holes;
	float		frequency_axis;
	float		frequency_axis_last;
	float		frequency_axis_fir;
}RearAxis;

struct
{
	float		oldValue;
	float		coeff[2];
}IIR_Filter;

struct
{

	int16_t	raw_accel_x;
	int16_t	raw_accel_y;
	int16_t raw_accel_z;
	int16_t	raw_gyro_z;

	int16_t FIR_raw_accel_x;
	int16_t	FIR_raw_accel_y;
	int16_t FIR_raw_accel_z;
	int16_t FIR_raw_gyro_z;

	float gyro_z;
	float accel_x;
	float accel_y;
	float accel_z;
	uint32_t speed;
	uint32_t speed_fir;
	uint32_t speed_iir_1;
	uint32_t countVal;  //Debug


}MotionData;

struct
{
	float	v_k;		//ergebniss der modells
	float 	v_k_1;
	float	v_k_f;		//gefilteretes ergebnis der geschwindigkeit
	float	v_m;		//messung der geschwindigkeit aus der raddrehzahl
	float	a_k;		//gemessene Beschleunigung

	float 	r_k;		//residuum/innovation
	float	alpha;
	float 	beta;


}estimation;
struct
{
	float	SP_rev;			//setpoint
	float	PV;				//stellwerr
	float 	PV_stell;		//stellwert nach begrenzung
	float	e_k;
	float	lastValue;
	float	sum;			//i regler summe
	uint32_t	e_k_1;
	uint32_t	t_1;
	uint32_t	t_2;
	uint32_t	T;
}velCtrl;


struct
{
	uint32_t	count;
	float		average[100];
	float		offset_accel_y;
	uint32_t	active;
	float		sum;
	uint32_t	imuCalibrated;
}offsetCalculation;

float buffer[5] = {0};			//Moving Average

int16_t	buffer_fir_accel_x[9] = {0};
int16_t	buffer_fir_accel_y[9] = {0};
int16_t	buffer_fir_accel_z[9] = {0};
int16_t	buffer_fir_gyro_z[9] = {0};

char	uart_rxBuffer[4] = {0};			//ascii eingagnspuffer 0 - 4000 1/min
uint32_t	_uart_rxBuffer = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM13_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */



/* Private function prototypes -----------------------------------------------*/

void getMotionSensorData();
void initMotorDriver();
void sendDiagnosisData();
float IIRFilter(float newValue, float fktOld, float fktNew);
void controller();
void init_IIR_Filter();
void setpoint_func();
void init_SpeedEstimation();
void mpuCalibration();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define REAR_GEAR_HOLES				10.0
#define TIMER_COUNTER_CYCLE			0.1		//Zykluszeit eines Timerdurchlaufes (90,000,000/50)/180,000
#define TIMER_CLOCK_PERIOD			900000.0	//ABP1 90 Mhz Timer Clk


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_TIM13_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */



 // HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  initMotorDriver();
  init_IIR_Filter();
  init_SpeedEstimation();


  HAL_I2C_Master_Transmit(&hi2c1, (MPU6500_ADDRESS_AD0_LOW << 1), (uint8_t *) &initData,sizeof(initData),200);


  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	//UART MAGIC NUMBER
	_GYRO_Z[0] = 0x63;
	_GYRO_Z[1] = 0x12;

	offsetCalculation.active = 1;

	HAL_UART_Receive_IT(&huart1, (uint8_t *) &uart_rxBuffer, sizeof(uart_rxBuffer));
  while (1)
  {

	  if (UART_SEND && UART_READY)
	  {
		  UART_SEND = 0;
		  UART_READY = 0;
		  j += 5;
		  char txBuffer[50] = {0};
		  sprintf(txBuffer, "%d,%d,%d,%d,\r\n",j, (int) MotionData.accel_y, (int) MotionData.speed_fir, (int) (estimation.v_k_f * 1000));
		  HAL_UART_Transmit_IT(&huart1, (uint8_t *) &txBuffer, sizeof(txBuffer));
	  }



	  if(UART_RX)
	  {
		  UART_RX = 0;


		  HAL_UART_Receive_IT(&huart1, (uint8_t *) &uart_rxBuffer, sizeof(uart_rxBuffer));

		  _uart_rxBuffer = atoi((char *) &uart_rxBuffer);

		  //velCtrl.SP_rev = _uart_rxBuffer * 1.0;
			  TEST_START = 1;


		  uart_rxBuffer[3] = 0;
		  uart_rxBuffer[2] = 0;
		  uart_rxBuffer[1] = 0;
		  uart_rxBuffer[0] = 0;

 		  //ok
	  }




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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 50-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 180000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 450;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 450-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM13 init function */
static void MX_TIM13_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 3;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 4000;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim13);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DRV_MODE2_Pin|DRV_PHASE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DRV_NSLEEP_Pin|DRV_MODE1_Pin|StatusLED2Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(StatusLEDPin_GPIO_Port, StatusLEDPin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DRV_MODE2_Pin */
  GPIO_InitStruct.Pin = DRV_MODE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DRV_MODE2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRV_PHASE_Pin */
  GPIO_InitStruct.Pin = DRV_PHASE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DRV_PHASE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV_NSLEEP_Pin DRV_MODE1_Pin StatusLED2Pin_Pin */
  GPIO_InitStruct.Pin = DRV_NSLEEP_Pin|DRV_MODE1_Pin|StatusLED2Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DRV_NFAULT_Pin */
  GPIO_InitStruct.Pin = DRV_NFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRV_NFAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : StatusLEDPin_Pin */
  GPIO_InitStruct.Pin = StatusLEDPin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(StatusLEDPin_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */






void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		UART_RX = 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
        UART_READY = 1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
        UART_READY = 1;
        UART_RX 	= 1;
}





void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance==TIM4)

	{					//Timer für das Senden der UART Daten
		setpoint_func();
		getMotionSensorData();
		controller();



	}
	else if (htim->Instance==TIM2)
	{
		TIMER_AXIS_FINISHED	= 1;

	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)
	{
			RearAxis.IC_Value					= TIM2->CCR1;
			__HAL_TIM_SET_COUNTER(&htim2,0);
			TIMER_AXIS_FINISHED = 0;
			RearAxis.faktor						= RearAxis.IC_Value / TIMER_CLOCK_PERIOD;
			RearAxis.period						= RearAxis.faktor * TIMER_COUNTER_CYCLE;
			RearAxis.frequency_holes			= (60 / RearAxis.period);		// 1 / min
			RearAxis.frequency_axis				= (RearAxis.frequency_holes / REAR_GEAR_HOLES);

			HAL_GPIO_TogglePin(StatusLEDPin_GPIO_Port, StatusLEDPin_Pin);

			if (RearAxis.frequency_axis > 4000)
			{
				RearAxis.frequency_axis = RearAxis.frequency_axis_last;
			}

			buffer[4] = buffer[3];
			buffer[3] = buffer[2];
			buffer[2] = buffer[1];
			buffer[1] = buffer[0];
			buffer[0] = RearAxis.frequency_axis ;		// 1/min

			RearAxis.frequency_axis_fir = (buffer[4] + buffer[3] + buffer[2] + buffer[1] + buffer[0]) / 5.0;
			RearAxis.frequency_axis_last = RearAxis.frequency_axis;
	}
}






void initMotorDriver()
{
	HAL_GPIO_WritePin(DRV_NSLEEP_GPIO_Port, DRV_NSLEEP_Pin, 1);
	HAL_GPIO_WritePin(DRV_PHASE_GPIO_Port, DRV_PHASE_Pin, 0);
	HAL_GPIO_WritePin(DRV_MODE1_GPIO_Port, DRV_MODE1_Pin, 1);
	HAL_GPIO_WritePin(DRV_MODE2_GPIO_Port, DRV_MODE2_Pin, 1);

}



void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM13)
	{
		MOTOR_READY = 1;
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//if(hi2c->Instance==I2C1)
	//{
		I2C_READY = 1;
	//}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//if(hi2c->Instance==I2C1)
	//{
		I2C_READY = 1;
	//}
}



void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{

  		I2C_READY = 1;

}

void setpoint_func()
{
	//in TIM4 zyklusszeit 200Hz / 5ms
		switch(setpointFunc.state)
		{
		case 0:
			setpointFunc.state = 1;
			break;
		case 1:
			setpointFunc.last_state  = 1;
			if (TEST_START)
			{
				setpointFunc.state = 2;
				break;
			}
			break;
		case 2:
			velCtrl.SP_rev = _uart_rxBuffer * 1.0;


			setpointFunc.last_state  = 2;
			setpointFunc.timerVal ++;

			if(setpointFunc.timerVal >= 200)		// 200 = 1s
			{
				setpointFunc.state = 3;
				setpointFunc.timerVal = 0;
				break;
			}
			break;
		case 3:

			if(setpointFunc.last_state == 2)
			{
				velCtrl.SP_rev = 0.0;
			}
			setpointFunc.last_state  = 3;
			setpointFunc.timerVal ++;

			if(setpointFunc.timerVal >= 200)			//60 = 600ms
			{
				setpointFunc.state = 2;
				setpointFunc.timerVal = 0;
				break;
			}
			break;

		}

}


void controller()
{
				//zyklisches rasussenden der Geschwindigkeitswerte, reset in der main

	if	(TIMER_AXIS_FINISHED == 0)
	{

			MotionData.speed	= RearAxis.frequency_axis;
			MotionData.speed_fir = RearAxis.frequency_axis_fir;

			buffer[0] = 0;
			buffer[1] = 0;
			buffer[2] = 0;
			buffer[3] = 0;
			buffer[4] = 0;
			buffer[5] = 0;
			buffer[6] = 0;
			buffer[7] = 0;
			buffer[8] = 0;
	}
	else
	{
		MotionData.speed	= 0;
		MotionData.speed_fir = 0;

	}
	UART_SEND = 1;

	//datenübertragung für estimatior

	estimation.a_k = MotionData.accel_y / 1000;
	estimation.v_m = MotionData.speed_fir * 0.0014833;





	//Schätzer

	if(offsetCalculation.imuCalibrated){
		estimation.v_k = estimation.v_k_1 + 0.005 * estimation.a_k;			//modell

		estimation.r_k = estimation.v_m - estimation.v_k;

		estimation.v_k_f = estimation.v_k + estimation.alpha * estimation.r_k;



		estimation.v_k_1 = estimation.v_k;
	}
	//regler
	/*
	  //velCtrl.SP_rev = 200.0;
	  	  velCtrl.sum	+= velCtrl.e_k;

		  velCtrl.e_k = (velCtrl.SP_rev - MotionData.speed_fir);
		  velCtrl.PV = velCtrl.lastValue + (2.1 * velCtrl.e_k) + 9.1 * 0.005 * velCtrl.sum;
		  // process = 1/min + ( 10 * ( 1/min - 1/min));

		  if (velCtrl.PV > 4000)
		  {
			  velCtrl.PV_stell = 4000;
		  }
		  else if(velCtrl.PV <= 0)
		  {
			  velCtrl.PV_stell = 0;
			  velCtrl.sum = 0;

		  }else{
			  velCtrl.PV_stell = velCtrl.PV;

		  }

*/

		  __HAL_TIM_SET_COMPARE(&htim13,TIM_CHANNEL_1, velCtrl.SP_rev);  //(int) velCtrl.PV_stell);
		  velCtrl.lastValue = RearAxis.frequency_axis_fir;
		  velCtrl.e_k_1		= velCtrl.e_k;





}


float IIRFilter(float newValue, float fktOld, float fktNew)
{
	return IIR_Filter.oldValue = fktNew * newValue + fktOld * IIR_Filter.oldValue;
}

void init_IIR_Filter()
{
	IIR_Filter.coeff[0] = 0.8;
	IIR_Filter.coeff[1] = 0.2;
	IIR_Filter.oldValue = 0;
}

void init_SpeedEstimation()
{
	estimation.alpha = 0.1;
}


void initMPU6500()
{
	uint8_t		wakeUpMPU6500[2]	= {MPU6500_RA_PWR_MGMT_1, 0x80 };
	uint16_t 	debugVar = 0;

	while(HAL_I2C_Master_Transmit_IT(&hi2c1, (MPU6500_ADDRESS_AD0_LOW << 1),(uint8_t *)  wakeUpMPU6500, sizeof(wakeUpMPU6500)) != HAL_OK)
	{
		debugVar++;
	}
	offsetCalculation.active = 1;			//kalibierroutine für mpu aktivieren

	HAL_Delay(1000);
}

void i2cScanner()
{
	volatile uint8_t hitList[100] = {0};
	uint8_t hitIndex = 0;
	uint8_t i2cdata = 0x3B;

    for (uint8_t devAdress = 0; devAdress < 255; devAdress++){

			while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
			{

			}

          if (HAL_I2C_Master_Receive_IT(&hi2c1,devAdress<<1,&i2cdata,sizeof(i2cdata)) == HAL_OK)
          {
                  hitList[hitIndex] = devAdress;
                  hitIndex++;
          }
    }


}

void getMotionSensorData()
{

	  if (I2C_READY ){			//|| (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
		  I2C_READY = 0;

		  if ((HAL_I2C_Mem_Read_IT(&hi2c1, (MPU6500_ADDRESS_AD0_LOW << 1),MPU6500_RA_ACCEL_XOUT_H,
		  	  		I2C_MEMADD_SIZE_8BIT, (uint8_t *) &ACCEL_XYZ, 6)) != HAL_OK )
		  {
			  MotionData.accel_x = 0;				// mm/s^2
			  		  MotionData.accel_y = 0;
			  		  MotionData.accel_z = 0;
			  		  DATA_READY_IMU = 1;
			  		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 500);

		  }else{

			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 0);

		 //MotionData.gyro_z = (((GYRO_Z[1] << 8) | GYRO_Z[0]) - 32767) / 131;


		  MotionData.raw_accel_x = ((ACCEL_XYZ[0] << 8) | ACCEL_XYZ[1]);
		  MotionData.raw_accel_y = ((ACCEL_XYZ[2] << 8) | ACCEL_XYZ[3]);
		  MotionData.raw_accel_z = ((ACCEL_XYZ[4] << 8) | ACCEL_XYZ[5]);

		  buffer_fir_accel_x[9] = buffer_fir_accel_x[8];
		  buffer_fir_accel_x[8] = buffer_fir_accel_x[7];
		  buffer_fir_accel_x[7] = buffer_fir_accel_x[6];
		  buffer_fir_accel_x[6] = buffer_fir_accel_x[5];
		  buffer_fir_accel_x[5] = buffer_fir_accel_x[4];
		  buffer_fir_accel_x[4] = buffer_fir_accel_x[3];
		  buffer_fir_accel_x[3] = buffer_fir_accel_x[2];
		  buffer_fir_accel_x[2] = buffer_fir_accel_x[1];
		  buffer_fir_accel_x[1] = buffer_fir_accel_x[0];
		  buffer_fir_accel_x[0] = MotionData.raw_accel_x;

		  buffer_fir_accel_y[9] = buffer_fir_accel_y[8];
		  buffer_fir_accel_y[8] = buffer_fir_accel_y[7];
		  buffer_fir_accel_y[7] = buffer_fir_accel_y[6];
		  buffer_fir_accel_y[6] = buffer_fir_accel_y[5];
		  buffer_fir_accel_y[5] = buffer_fir_accel_y[4];
		  buffer_fir_accel_y[4] = buffer_fir_accel_y[3];
		  buffer_fir_accel_y[3] = buffer_fir_accel_y[2];
		  buffer_fir_accel_y[2] = buffer_fir_accel_y[1];
		  buffer_fir_accel_y[1] = buffer_fir_accel_y[0];
		  buffer_fir_accel_y[0] = MotionData.raw_accel_y;

		  MotionData.FIR_raw_accel_x = (  buffer_fir_accel_x[8] +
										  buffer_fir_accel_x[7] +
										  buffer_fir_accel_x[6] +
										  buffer_fir_accel_x[5] +
										  buffer_fir_accel_x[4] +
										  buffer_fir_accel_x[3] +
										  buffer_fir_accel_x[2] +
										  buffer_fir_accel_x[1] +
										  buffer_fir_accel_x[0] ) / 8.0;

		  MotionData.FIR_raw_accel_y = (  buffer_fir_accel_y[8] +
										  buffer_fir_accel_y[7] +
										  buffer_fir_accel_y[6] +
										  buffer_fir_accel_y[5] +
										  buffer_fir_accel_y[4] +
										  buffer_fir_accel_y[3] +
										  buffer_fir_accel_y[2] +
										  buffer_fir_accel_y[1] +
										  buffer_fir_accel_y[0] ) / 8.0;

		  MotionData.accel_x = MotionData.FIR_raw_accel_x / 1.6384;				// mm/s^2
		  MotionData.accel_y = (MotionData.FIR_raw_accel_y / 1.6384) - offsetCalculation.offset_accel_y;
		  MotionData.accel_z = MotionData.raw_accel_z / 1.6384;
		  DATA_READY_IMU = 1;
		  //MotionData.accel_y = (((ACCEL_XY[2] << 8) | ACCEL_XY[3])- 32767.0) / 131;

		  mpuCalibration();
		  }

	 }
}

void mpuCalibration()
{
	if (offsetCalculation.active){


		if(offsetCalculation.count <= 99){
			offsetCalculation.average[offsetCalculation.count] = MotionData.accel_y;
			offsetCalculation.count ++;
		}else{
			if(a<=99)
			{
				offsetCalculation.sum += offsetCalculation.average[a];
				a++;

			}else{
				offsetCalculation.offset_accel_y = offsetCalculation.sum / 99.0;
				offsetCalculation.active = 0;
				offsetCalculation.imuCalibrated = 1;
			}
		}
	}
}


/*
void SM_IMU()
{
	switch(STATE_IMU)
	{
	case 0:


		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	case 4:
		break;

	}
}
*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
