/**
  ******************************************************************************
  * @file    SPI/SPI_FullDuplex_ComPolling/Inc/main.h
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    29-April-2016
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#define XL362_REG_WRITE   0x0a
#define XL362_REG_READ    0x0b
#define XL362_FIFO_READ   0x0d

/* #define dump from Analog Devices' sample code begins */
/*************************************************************************************/
#define XL362_DEVID_AD           0x00
#define XL362_DEVID_MST          0x01
#define XL362_PARTID             0x02
#define XL362_REVID              0x03
#define XL362_XID                0x04
#define XL362_ID                 0xe2

/* -- RO data -- */
#define XL362_XDATA8             0x08
#define XL362_YDATA8             0x09
#define XL362_ZDATA8             0x0A
#define XL362_STATUS             0x0B
#define XL362_FIFO_ENTRIES_L     0x0C
#define XL362_FIFO_ENTRIES_H     0x0D
#define XL362_XDATAL             0x0e
#define XL362_XDATAH             0x0f
#define XL362_YDATAL             0x10
#define XL362_YDATAH             0x11
#define XL362_ZDATAL             0x12
#define XL362_ZDATAH             0x13
#define XL362_TEMPL              0x14
#define XL362_TEMPH              0x15
#define XL362_X_ADCL             0x16
#define XL362_X_ADCH             0x17

/* -- Control and Config -- */
#define XL362_SOFT_RESET         0x1f
#define XL362_THRESH_ACTL        0x20
#define XL362_THRESH_ACTH        0x21
#define XL362_TIME_ACT           0x22
#define XL362_THRESH_INACTL      0x23
#define XL362_THRESH_INACTH      0x24
#define XL362_TIME_INACTL        0x25
#define XL362_TIME_INACTH        0x26
#define XL362_ACT_INACT_CTL      0x27
#define XL362_FIFO_CONTROL       0x28
#define XL362_FIFO_SAMPLES       0x29
#define XL362_INTMAP1            0x2a
#define XL362_INTMAP2            0x2b
#define XL362_FILTER_CTL         0x2c
#define XL362_POWER_CTL          0x2d
#define XL362_SELF_TEST          0x2e

/******************************************************************************/
/* The Configuration Values for the ADXL362's registers */
#define GTHRESH_5MG             0x05
#define INACTIVE_THRESH_1MG     0x01
#define MIN_TIME_1MS            0x01
#define TOTALLY_INACTIVE        0xFF
#define ENABLE_ONLY             0x01
#define OLDEST_SAVED            0x0C
#define FULL_4_SAMPLES          0x80
#define ODR200_HZ               0x04
#define MEASURE_MODE            0x02
#define ACT_THRESH_H            0x00
#define INACT_THRESH_H          0x00
/* Includes ------------------------------------------------------------------*/
#include "stm32vl_discovery.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor SPIx instance used and associated
   resources */
/* Definition for SPIx clock resources */
#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_5
#define SPIx_SCK_GPIO_PORT               GPIOA
#define SPIx_MISO_PIN                    GPIO_PIN_6
#define SPIx_MISO_GPIO_PORT              GPIOA
#define SPIx_MOSI_PIN                    GPIO_PIN_7
#define SPIx_MOSI_GPIO_PORT              GPIOA
#define SPIx_CS_PIN                      GPIO_PIN_8
#define SPIx_CS_GPIO_PORT                GPIOA

/* Size of buffer */
#define BUFFERSIZE                       3
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
