/**
  ******************************************************************************
  * @file    SPI/SPI_FullDuplex_ComPolling/Src/main.c
  * @author  MCD Application Team (edited by Kate Andrew, Dynamic Controls Intern)
  * @version V1.4.0
  * @date    29-April-2016 (last date edited 16-January-2017)
  * @brief   Sample code for communication between two STM32 Discovery boards 
             edited to allow for communication between an EVAL-ADXL362-Z 
             evaluation board and an STM32 Discovery board.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

/* This example program has been edited in order to collect acceleration data from the ADXL362. Acceleration data resolution is
1000 bits per g according to the ADXL362 datasheet, but may be slightly more than this. The on-board temperature sensor is also
utilised, with its data being read at the same time as the acceleration data was gathered. The data from the temperature sensor
is assumed to be provided at a resolution of 1 bit per degree Kelvin. */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>

/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup SPI_FullDuplex_ComPolling
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Uncomment this line to use the board as master, if not it is used as slave */
#define MASTER_BOARD


/* Ends the pasting of #defines from the Analog Devices Sample Code */
/*************************************************************************************/

/* Private variables ---------------------------------------------------------*/
/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;

/* Buffers used for transmission and reception */
uint8_t aTxBuffer[];
uint8_t aRxBuffer[];

// Watch Variables
uint8_t status;
uint8_t xHigh;
uint8_t xLow;
uint8_t yHigh;
uint8_t yLow;
uint8_t zHigh;
uint8_t zLow;
uint8_t tempLow;
uint8_t tempHigh;
int16_t xEmpty = 0x0000;
int16_t yEmpty = 0x0000;
int16_t zEmpty = 0x0000;
int16_t tempEmpty = 0x0000; 
int16_t bitcheck = 0xFF00;
int16_t xFullFirst;
int16_t yFullFirst;
int16_t zFullFirst;
int16_t tempFullFirst;
int16_t xFull;
int16_t yFull;
int16_t zFull;
int16_t tempFull;
uint8_t count = 0; // Number of samples to be collected per while loop. 
int16_t xArray[50];
int16_t yArray[50];
int16_t zArray[50];
int16_t tempArray[50];
int16_t* Xarray = xArray;
int16_t* Yarray = yArray;
int16_t* Zarray = zArray;
int16_t* Tarray = tempArray;
FILE *fileptr;



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
/* static void Timeout_Error_Handler(void);
static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength); */
void SPIx_CS_Init(void);
int8_t spiDataRead(uint8_t command, uint8_t reg);
int8_t spiDataWrite(uint8_t command, uint8_t reg, uint8_t data);
int8_t spiFifoRead(uint8_t command);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  command, uint8_t reg if using a read. Command, uint8_t reg and data if using a write. If using a read/write then two 
    commands, two registers and one data byte must be used as parameters. 
  * @retval Haven't figured out how to make it anything worth having. 
  */
/* a function that has the command for the ADXL362 and the register involved as inputs, the TransmitReceive function is then
called using the command and register address as entries in the transmission buffer. Chip Select is rest, 
becoming low at the start of the function, and then set at the end of the function. */

// Function for writing values to registers on the ADXL362
int8_t spiDataWrite(uint8_t command, uint8_t reg, uint8_t data)
{
  uint8_t aTxBuffer[] = {command, reg, data}; // buffer used for transmission
  int8_t aRxBuffer[BUFFERSIZE];
  HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_RESET); // chip select low- signal that transmission can be started
  HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE, 5000); //
  HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_SET); // chip select high- signal that transmission is finished
  return(aRxBuffer[2]);
}

// Function for reading values from registers on the ADXL362
int8_t spiDataRead(uint8_t command, uint8_t reg)
{
  uint8_t aTxBuffer[] = {command, reg}; // buffer used for transmission
  int8_t aRxBuffer[BUFFERSIZE];
  HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_RESET); // chip select low- signal that transmission can be started
  HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE, 5000); //
  HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_SET); // chip select high- signal that transmission is finished
  return(aRxBuffer[2]);
}

// Function for reading the ADXL362's FIFO buffer. Not used in this program but could be useful if the Stream mode is used.
int8_t spiFifoRead(uint8_t command)
{
  uint8_t aTxBuffer[] = {command}; // buffer used for transmission
  int8_t aRxBuffer[BUFFERSIZE];
  HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_RESET); // chip select low- signal that transmission can be started
  HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE, 5000); //
  HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_SET); // chip select high- signal that transmission is finished
  return(aRxBuffer[2]);
}
// End of Private Functions

int main(void)
{
  /* STM32F1xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();
  
  /* Configure the system clock to 24 MHz */
  SystemClock_Config();

  /* Configure LED3 and LED4 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  SPIx_CS_Init();
 

  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPIx;
  
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.NSS               = SPI_NSS_HARD_OUTPUT;
  SpiHandle.pTxBuffPtr             = aTxBuffer;
  SpiHandle.pRxBuffPtr             = aRxBuffer;
  
#ifdef MASTER_BOARD
  SpiHandle.Init.Mode = SPI_MODE_MASTER;
#else
  SpiHandle.Init.Mode = SPI_MODE_SLAVE;
#endif /* MASTER_BOARD */

  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

#ifdef MASTER_BOARD
  /* SPI block is enabled prior calling SPI transmit/receive functions, in order to get CLK signal properly pulled down.
     Otherwise, SPI CLK signal is not clean on this board and leads to errors during transfer */
  __HAL_SPI_ENABLE(&SpiHandle);

  /* Configure User push-button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
  /* Wait for User push-button press before starting the Communication */
  while (BSP_PB_GetState(BUTTON_USER) != GPIO_PIN_SET)
  {
    BSP_LED_Toggle(LED3);
    HAL_Delay(100);
  }
  BSP_LED_Off(LED3);
#endif /* MASTER_BOARD */

  // Configure the ADXL362, see datasheet for more information, page 36 "Device Configuration"
  
  spiDataWrite(XL362_REG_WRITE, XL362_THRESH_ACTL, GTHRESH_5MG); // Setting the threshold to 5 mg so that the device is "always on"
  spiDataWrite(XL362_REG_WRITE, XL362_THRESH_ACTH, ACT_THRESH_H);
  spiDataWrite(XL362_REG_WRITE, XL362_TIME_ACT, MIN_TIME_1MS); // Even if only 1 sample is picked up with at least one reading over zero, it will be noticed.
  spiDataWrite(XL362_REG_WRITE, XL362_THRESH_INACTL, INACTIVE_THRESH_1MG);
  spiDataWrite(XL362_REG_WRITE, XL362_THRESH_INACTH, INACT_THRESH_H);
  spiDataWrite(XL362_REG_WRITE, XL362_TIME_INACTL, TOTALLY_INACTIVE);
  spiDataWrite(XL362_REG_WRITE, XL362_TIME_INACTH, TOTALLY_INACTIVE);
  spiDataWrite(XL362_REG_WRITE, XL362_ACT_INACT_CTL, ENABLE_ONLY); // activity enable set
  spiDataWrite(XL362_REG_WRITE, XL362_FIFO_CONTROL, OLDEST_SAVED); // FIFO in oldest saved, includes temperature measurement and Above Half enabled
  spiDataWrite(XL362_REG_WRITE, XL362_FIFO_SAMPLES, FULL_4_SAMPLES); // 128 samples to be stored in the buffer. 
  spiDataWrite(XL362_REG_WRITE, XL362_FILTER_CTL, ODR200_HZ); // output data rate chosen as 200 Hz
  spiDataWrite(XL362_REG_WRITE, XL362_POWER_CTL, MEASURE_MODE); // puts the device in measurement mode 
  
  
  fileptr = fopen("position12.txt", "w"); // opening the log file in "write" mode
  // for loop gathering 150 samples for the X, Y and Z axes and the on-board temperature sensor
  for (count = 0; count < 50; count++)
  {
    HAL_SPI_Init(&SpiHandle); 
    xHigh = spiDataRead(XL362_REG_READ, XL362_XDATAH); // first four bits of the 12 bit reading for acceleration in the X direction
    xLow = spiDataRead(XL362_REG_READ, XL362_XDATAL);  // Least significant byte for acceleration in the X direction 
    yHigh = spiDataRead(XL362_REG_READ, XL362_YDATAH); // first four bits of the 12 bit reading for acceleration in the Y direction
    yLow = spiDataRead(XL362_REG_READ, XL362_YDATAL);  // Least significant byte for acceleration in the Y direction
    zHigh = spiDataRead(XL362_REG_READ, XL362_ZDATAH); // first four bits of the 12 bit reading for acceleration in the Z direction
    zLow = spiDataRead(XL362_REG_READ, XL362_ZDATAL);  // Least significant byte for acceleration in the Z direction
    tempHigh = spiDataRead(XL362_REG_READ, XL362_TEMPH); // first four bits of the 12 bit temperature reading
    tempLow = spiDataRead(XL362_REG_READ, XL362_TEMPL);  // Least significant byte of the temperature reading
    
    // Converting the values from two 8-bit ints to one 16-bit int  
    // Bit shifting and checking the MSB to ensure the remaining space is being padded with zeroes. 
    xFullFirst = (xEmpty | (xHigh << 8)) & bitcheck; 
    yFullFirst = (yEmpty | (yHigh << 8)) & bitcheck;
    zFullFirst= (zEmpty | (zHigh << 8)) & bitcheck;
    tempFullFirst = (tempEmpty | (tempHigh << 8)) & bitcheck;
    // Using the bitwise or to add on the LSB values
    xFull = xFullFirst | xLow;
    yFull = yFullFirst | yLow;
    zFull = zFullFirst | zLow;
    tempFull = tempFullFirst | tempLow;
    
    // Reading the status register and placing the Acceleration and Temperature Values into their respective arrays
    status = spiDataRead(XL362_REG_READ, XL362_STATUS);
    xArray[count] = xFull;
    yArray[count] = yFull;
    zArray[count] = zFull;
    tempArray[count] = tempFull; 
    fprintf(fileptr, "%d, %i, %i, %i, %i \n",count, xArray[count], yArray[count], zArray[count], tempArray[count]);
  }
 fclose(fileptr);
}

// Function for manually driving the Chip-Select pin
void SPIx_CS_Init(void)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};
  
  /* Enable the GPIO_PORT_A clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure the GPIO_CS pin */
  gpioinitstruct.Pin    = SPIx_CS_PIN;
  gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull   = GPIO_NOPULL;
  gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPIx_CS_GPIO_PORT, &gpioinitstruct);

  /* Set PIN to return CS to HIGH state*/
  HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_SET);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
/* static void Timeout_Error_Handler(void)
{
  Toggle LED4 on 
  while(1)
  {
    BSP_LED_On(LED4);
    HAL_Delay(500);
    BSP_LED_Off(LED4);
    HAL_Delay(500);
  }
} */

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 24000000
  *            HCLK(Hz)                       = 24000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV1                    = 2
  *            PLLMUL                         = 6
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV2;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_0)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
