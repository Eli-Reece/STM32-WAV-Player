/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "fatfs.h"
#include "i2s.h"
#include "spi.h"r
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
/* USER CODE END Includes */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void qprint(const char *fmt, ...);
void qprint(const char *fmt, ...)
{
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Globals /////////////////////////////////////////////////////////////////////
FATFS fatfs;
FRESULT fres;
FIL fil;
UINT bytesRead;

// For 44.1kHz stereo, one frame is 2 samples (left + right channel)
#define SAMPLES_PER_FRAME 2
#define SAMPLE_RATE 44100
#define BUFFER_TIME_MS 20  // 20ms buffer is a good compromise

// Calculate buffer size based on sample rate
#define AUDIO_BUFFER_SIZE ((SAMPLE_RATE * BUFFER_TIME_MS * SAMPLES_PER_FRAME) / 1000)
#define HALF_BUFFER_SIZE (AUDIO_BUFFER_SIZE/2)

int16_t samples[AUDIO_BUFFER_SIZE];

uint32_t fread_size = 0;
uint32_t recording_size = 0;
uint32_t played_size = 0;

volatile bool isHalfBufferDone = false;
volatile bool isFullBufferDone = false;

// Callbacks ///////////////////////////////////////////////////////////////////
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	isHalfBufferDone = true;
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	isFullBufferDone = true;
    played_size += AUDIO_BUFFER_SIZE;
}

// Main ////////////////////////////////////////////////////////////////////////
int main(void)
{
    // Abstraction Layer and System Clock
    HAL_Init();
    SystemClock_Config();

    // Peripheral Init
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_DMA_Init();
    MX_FATFS_Init();
    MX_SPI2_Init();
    MX_I2S1_Init();

    // Delay for SD Card
    HAL_Delay(1000);

    // Open the file system
    fres = f_mount(&fatfs, "", 1);
    if (fres != FR_OK) {
        qprint("f_mount error (%i)\r\n", fres);
    }

    // Open the Song
    fres = f_open(&fil, "test.wav", FA_READ);

    // Get song header data
    uint8_t header[44];
    f_read(&fil, header, 44, &bytesRead);
    qprint("Sample Rate: %d\r\n", *(uint32_t*)(header + 24));
    qprint("Bits Per Sample: %d\r\n", *(uint16_t*)(header + 34));
    qprint("Channels: %d\r\n", *(uint16_t*)(header + 22));

    // Get Song Size
    f_lseek(&fil, 40);
    f_read(&fil, &recording_size, 4, &bytesRead);
    qprint("Recording Size: %d\r\n", recording_size);

    // Play the Song
    f_read(&fil, samples, AUDIO_BUFFER_SIZE * sizeof(uint16_t), &bytesRead);
    HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t *)samples, AUDIO_BUFFER_SIZE);

    while(1)
    {
        if (isHalfBufferDone) {
            f_read(&fil, samples, HALF_BUFFER_SIZE * sizeof(uint16_t), &bytesRead);
            isHalfBufferDone = false;
        }

        if (isFullBufferDone) {
            f_read(&fil, &samples[HALF_BUFFER_SIZE], HALF_BUFFER_SIZE * sizeof(uint16_t), &bytesRead);
            isFullBufferDone = false;
        }

        if (played_size >= recording_size) {
            HAL_I2S_DMAStop(&hi2s1);
        }
    }

    // Unmount the FS
    f_mount(NULL, "", 0);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
