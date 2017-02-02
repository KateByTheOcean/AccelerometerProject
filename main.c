/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Program for analog to digital conversion from the 
                         ADXL335 and the KXTC9-2050 accelerometers using the 
                         STM32F100 Value Line discovery board as the MCU. 
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

ADC_HandleTypeDef hadc2;

ADC_HandleTypeDef hadc3;

ADC_HandleTypeDef hadc4;

ADC_HandleTypeDef hadc5;

CRC_HandleTypeDef hcrc;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;



/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* Variables for the ADC values of the Acceleration, and their values in gs */
int count = 0;

/* If ADXL335 is being used, 
these values should be used for the sensitivity (bits/g) and the zero g offset 
float ZEROPOINTZ = 2047; 
float ZEROPOINTX = 1960;
float ZEROPOINTY = 1960;
float SCALEFACTOR = 4095;
float BITRANGEX = 409;
float BITRANGEY = 409;
float BITRANGEZ = 409;
float VOLTAGE = 3000; 
*/

/* If KXTC9-2050 is being used, 
these values should be used for the sensitivity (bits/g) and the zero g offset*/
int ZEROPOINTZ = 1975;
int ZEROPOINTY = 1975;
int ZEROPOINTX = 1975;
int SCALEFACTOR = 8195;
float BITRANGEX = 819;
float BITRANGEY = 819;
float BITRANGEZ = 819;
int VOLTAGE = 3000; 

// Watch List Variables
// arrays holding 255 samples for data for each axis
short xArray[50];
short yArray[50];
short zArray[50]; 
short vRefArray[50];
short TempArray[50];

float newGx[50];
float newGy[50];
float newGz[50];

float XChannelVoltage[50];
float YChannelVoltage[50];
float ZChannelVoltage[50];

short* Xarray = xArray;
short* Yarray = yArray;
short* Zarray = zArray;
short* vRefPoint = vRefArray;
short* TarrayPoint = TempArray;

float* gX = newGx;
float* gY = newGy;
float* gZ = newGz;

float* mVX = XChannelVoltage;
float* mVY = YChannelVoltage;
float* mVZ = ZChannelVoltage;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void); 
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
static void MX_ADC5_Init(void);
static void MX_CRC_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init(); /* hardware abstraction layer code call */

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init(); 
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_ADC5_Init();
  MX_CRC_Init();
  MX_IWDG_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  
  for (count = 0; count < 50; count++)
  {
    /************************************************************************************/
    /* Reading the acceleration values for the X axis */
    /************************************************************************************/
    ADC_ChannelConfTypeDef sConfig1;
    
    sConfig1.Channel = ADC_CHANNEL_4;
    sConfig1.Rank = 1;
    sConfig1.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig1) != HAL_OK)
    {
      Error_Handler();
    } 
    
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    xArray[count] = HAL_ADC_GetValue(&hadc1);
    newGx[count] = ((xArray[count] - ZEROPOINTX) + 1)/BITRANGEX;
    
    /************************************************************************************/
    /* Reading the acceleration values for the Y axis */
    /************************************************************************************/
    ADC_ChannelConfTypeDef sConfig2;
    
    sConfig2.Channel = ADC_CHANNEL_5;
    sConfig2.Rank = 1;
    sConfig2.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig2) != HAL_OK)
    { 
      Error_Handler();
    }
    
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 100); 
    yArray[count] = HAL_ADC_GetValue(&hadc2);
    // adcValmVy = (adcValy-ZEROPOINTY)/SCALEFACTOR * VOLTAGE; 
    newGy[count] = ((yArray[count] - ZEROPOINTY) + 1)/BITRANGEY;
    // adcValmVx = (adcValx - ZEROPOINTX)/SCALEFACTOR * VOLTAGE; 
    
    /************************************************************************************/
    /* Reading the acceleration values for the Z axis */
    /************************************************************************************/
    ADC_ChannelConfTypeDef sConfig3;
    
    sConfig3.Channel = ADC_CHANNEL_6;
    sConfig3.Rank = 1;
    sConfig3.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig3) != HAL_OK)
    { 
      Error_Handler();
    }
    
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 100); 
    zArray[count] = HAL_ADC_GetValue(&hadc3); 
    newGz[count] = ((zArray[count] - ZEROPOINTZ) + 1)/BITRANGEZ; 
    
    /************************************************************************************/
    /* Reading the value (in bits) of the STM32's internal reference voltage */
    /************************************************************************************/
    ADC_ChannelConfTypeDef sConfig4;
    
    sConfig4.Channel = ADC_CHANNEL_VREFINT;
    sConfig4.Rank = 1;
    sConfig4.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc4, &sConfig4) != HAL_OK)
    { 
      Error_Handler();
    }
    
    HAL_ADC_Start(&hadc4);
    HAL_ADC_PollForConversion(&hadc4, 100);
    vRefArray[count] = HAL_ADC_GetValue(&hadc4);
    
    /************************************************************************************/
    /* Reading the output (in bits) of the STM32's Temperature Sensor */
    /************************************************************************************/
    ADC_ChannelConfTypeDef sConfig5;
    
    sConfig5.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig5.Rank = 1;
    sConfig5.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc5, &sConfig5) != HAL_OK)
    { 
      Error_Handler();
    }
    
    HAL_ADC_Start(&hadc5);
    HAL_ADC_PollForConversion(&hadc5, 100);
    TempArray[count] = HAL_ADC_GetValue(&hadc5);
    
    /***********************************************************************************/
    /* Using VREFINT to find the values of X, Y and Z output in mV.
    The value of the actual voltage of VREFINT was manually calculated and added 
    into the formula, its specified value is 1.2 V but it can be anything between 1.16V
    and 1.26 V. */
    /***********************************************************************************/
    XChannelVoltage[count] = xArray[count] * 1164/vRefArray[count];
    YChannelVoltage[count] = yArray[count] * 1164/vRefArray[count];
    ZChannelVoltage[count] = zArray[count] * 1164/vRefArray[count];
    
  }

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
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


/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig1; 
  
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  
  sConfig1.Channel = ADC_CHANNEL_4;
  sConfig1.Rank = 1;
  sConfig1.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig1) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_ADC2_Init(void)
{ 
  ADC_ChannelConfTypeDef sConfig2;
  
  hadc2.Instance = ADC1;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  
  sConfig2.Channel = ADC_CHANNEL_5;
  sConfig2.Rank = 1;
  sConfig2.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC3_Init(void)
{ 
  ADC_ChannelConfTypeDef sConfig3;
  
  hadc3.Instance = ADC1;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  
  sConfig3.Channel = ADC_CHANNEL_6;
  sConfig3.Rank = 1;
  sConfig3.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig3) != HAL_OK)
  {
    Error_Handler();
  }
}
         
static void MX_ADC4_Init(void)
{ 
  ADC_ChannelConfTypeDef sConfig4;
  
  hadc4.Instance = ADC1;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.ContinuousConvMode = ENABLE;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }
  
  sConfig4.Channel = ADC_CHANNEL_VREFINT;
  sConfig4.Rank = 1;
  sConfig4.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig4) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC5_Init(void)
{ 
  ADC_ChannelConfTypeDef sConfig5;
  
  hadc5.Instance = ADC1;
  hadc5.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc5.Init.ContinuousConvMode = ENABLE;
  hadc5.Init.DiscontinuousConvMode = DISABLE;
  hadc5.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc5.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc5.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc5) != HAL_OK)
  {
    Error_Handler();
  }
  
  sConfig5.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig5.Rank = 1;
  sConfig5.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc5, &sConfig5) != HAL_OK)
  {
    Error_Handler();
  }
}

    /**Configure Regular Channel 
    */


/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_TRC;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_CTS;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}


static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  /* Turn LED4 on */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */ 
}



#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
