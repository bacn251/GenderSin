/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2s.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ============================================================================
// OPTIMIZED SINE WAVE GENERATION FOR I2S DMA
// ============================================================================

#define SAMPLE_RATE   96000
#define SINE_FREQ     5000.0f
#define DMA_SAMPLES   256

// Precomputed constants for optimization
#define TWO_PI        6.28318530717959f
#define PHASE_INC     (TWO_PI * SINE_FREQ / SAMPLE_RATE)
#define AMPLITUDE_24  8388607.0f  // Max 24-bit signed value

// Double buffer for circular DMA mode (aligned for optimal DMA performance)
int32_t i2s_tx_buf[DMA_SAMPLES * 2 * 2] __attribute__((aligned(4)));

// Global phase accumulator (volatile for ISR safety)
static volatile float phase = 0.0f;

/**
 * @brief Fill half of the I2S buffer with sine wave samples
 * @param buf: Pointer to the buffer
 * @param offset: Starting offset in the buffer (0 or DMA_SAMPLES*2)
 * @note Optimized for MCU: uses local variables, minimizes function calls
 */
void Fill_Sine_Buffer_Half(int32_t *buf, uint32_t offset)
{
    // Local copy for faster access (avoid volatile reads in loop)
    float local_phase = phase;
    
    // Generate samples for one half of the buffer
    for (uint32_t i = 0; i < DMA_SAMPLES; i++)
    {
        // Generate sine sample
        float sine_val = sinf(local_phase);
        
        // Update phase with wraparound
        local_phase += PHASE_INC;
        if (local_phase >= TWO_PI)
            local_phase -= TWO_PI;
        
        // Convert to 24-bit integer, then shift to MSB of 32-bit word
        int32_t sample24 = (int32_t)(sine_val * AMPLITUDE_24);
        int32_t sample32 = sample24 << 8;  // I2S expects 24-bit in upper bits
        
        // Write stereo samples (same value for L and R)
        uint32_t idx = offset + (i << 1);  // Faster than i*2
        buf[idx]     = sample32;  // Left channel
        buf[idx + 1] = sample32;  // Right channel
    }
    
    // Update global phase for continuity
    phase = local_phase;
}

/**
 * @brief DMA Half Transfer Complete Callback
 * @note Called when DMA finishes transmitting FIRST HALF of buffer
 *       We must refill the first half while DMA transmits second half
 */
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2)  // Ensure it's our I2S instance
    {
        Fill_Sine_Buffer_Half(i2s_tx_buf, 0);  // Refill first half
    }
}

/**
 * @brief DMA Transfer Complete Callback
 * @note Called when DMA finishes transmitting SECOND HALF of buffer
 *       We must refill the second half while DMA wraps to first half
 */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2)  // Ensure it's our I2S instance
    {
        Fill_Sine_Buffer_Half(i2s_tx_buf, DMA_SAMPLES * 2);  // Refill second half
    }
}

void I2S_Start_TX(void)
{
    // Fill cả buffer ban đầu
    Fill_Sine_Buffer_Half(i2s_tx_buf, 0);
    Fill_Sine_Buffer_Half(i2s_tx_buf, DMA_SAMPLES * 2);

    // Start với CIRCULAR mode
    HAL_I2S_Transmit_DMA(
        &hi2s2,
        (uint16_t *)i2s_tx_buf,
        DMA_SAMPLES * 2 * 2 // Total samples (L+R) for both halves
    );
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
  MX_DMA_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */
  I2S_Start_TX();
  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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
#ifdef USE_FULL_ASSERT
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
