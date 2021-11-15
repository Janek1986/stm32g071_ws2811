/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "ws2811.h"
#include "ws2811_fx.h"
#include "stm32g0xx_hal_spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//uint8_t bufferTx[2] = {60,61};
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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  WS2811_Init(&hspi2);

  WS2811FX_Init(1);	// Start 3 segments

  WS2811FX_SetSpeed(0, 250);	// Speed of segment 0
//  WS2811FX_SetSpeed(1, 2000);	// Speed of segment 1
//  WS2811FX_SetSpeed(2, 500);	// Speed of segment 2
//  WS2811FX_SetColorRGB(0, 32,0,64);	// Set color 0
//  WS2811FX_SetColorRGB(1, 32,0,0);		// Set color 1
//  WS2811FX_SetColorRGB(2, 0,64,0);		// Set color 2
//  WS2811FX_SetMode(0, FX_MODE_WHITE_TO_COLOR);	// Set mode segment 0

//  WS2811FX_SetColorRGB(0, 16,64,0);
//  WS2811FX_SetColorRGB(1, 0,32,64);
//  WS2811FX_SetColorRGB(2, 64,0,0);
//  WS2811FX_SetMode(0, FX_MODE_BLACK_TO_COLOR);	// Set mode segment 1

  WS2811FX_SetColorRGB(0, 16,64,0);
  WS2811FX_SetColorRGB(1, 0,32,64);
  WS2811FX_SetColorRGB(2, 64,0,0);
  WS2811FX_SetMode(0, FX_MODE_COLOR_WIPE); 	// Set mode segment 2

  WS2811FX_Start(0);	// Start segment 0
//  WS2811FX_Start(1);	// Start segment 1
//  WS2811FX_Start(2);	// Start segment 2
  HAL_Delay(200);
 // WS2811_Refresh();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //uint8_t index = 0;
  while (1)
  {
		//HAL_SPI_Transmit_DMA(&hspi2, bufferTx, 2); // Additional 3 for reset signal
		//while(HAL_DMA_STATE_READY != HAL_DMA_GetState(hspi2.hdmatx));

//	  switch (index) {
//	  case 0:
//		  WS2811_SetDiodeColor(2, 0x00000000);
//		  WS2811_SetDiodeColor(0, 0x000000FF);	// Set color 0
//		  index++;
//		  break;
//	  case 1:
//		  WS2811_SetDiodeColor(0, 0x0000FF00);	// Set color 0
//		  index++;
//		  break;
//	  case 2:
//		  WS2811_SetDiodeColor(0, 0x00FF0000);	// Set color 0
//		  index++;
//		  break;
//	  case 3:
//		  WS2811_SetDiodeColor(0, 0x00000000);
//		  WS2811_SetDiodeColor(1, 0x000000FF);	// Set color 0
//		  index++;
//		  break;
//	  case 4:
//		  WS2811_SetDiodeColor(1, 0x0000FF00);	// Set color 0
//		  index++;
//		  break;
//	  case 5:
//		  WS2811_SetDiodeColor(1, 0x00FF0000);	// Set color 0
//		  index++;
//		  break;
//	  case 6:
//		  WS2811_SetDiodeColor(1, 0x00000000);
//		  WS2811_SetDiodeColor(2, 0x000000FF);	// Set color 0
//		  index++;
//		  break;
//	  case 7:
//		  WS2811_SetDiodeColor(2, 0x0000FF00);	// Set color 0
//		  index++;
//		  break;
//	  case 8:
//		  WS2811_SetDiodeColor(2, 0x00FF0000);	// Set color 0
//		  index = 0;
//		  break;
//	  }


//	  WS2811FX_Start(0);	// Start segment 0
	  WS2811FX_Callback();	// FX effects calllback
//	  HAL_GPIO_TogglePin(WSout_GPIO_Port, WSout_Pin);
//	  WS2811_Refresh();
//	  HAL_Delay(200);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void)
{
	WS2811FX_SysTickCallback();	// FX effects software timers
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
