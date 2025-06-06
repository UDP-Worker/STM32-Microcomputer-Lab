/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

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
 TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define RX_BUF_LEN   16          /* "T235959\r\n" 只需 9 字节，留裕量 */
volatile uint8_t  rx_buf[RX_BUF_LEN];
volatile uint8_t  rx_len = 0;    /* 已收字节数 */
volatile uint8_t  rx_ready = 0;  /* 一帧完毕标志 */


const uint8_t LED_CODE[] = {
  0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,
  0x80,0x90,0x88,0x83,0xC6,0xA1,0x86,0x8E,
  0xBF
};
uint32_t time_in_second = 0;
uint8_t LED_BUF[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t disp_ind = 0;
volatile uint8_t rx_frame_len = 0;
volatile uint8_t tx_time_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void out_595(uint8_t data)
{
  for (int i = 0; i < 8; i++) {
    HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET);
    if (data & (0x80 >> i))
      HAL_GPIO_WritePin(DIO_GPIO_Port, DIO_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(DIO_GPIO_Port, DIO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET);
  }
}

static void refresh_led_from_time(void)
{
  uint32_t hours = (time_in_second/3600) % 24;
  uint32_t minutes = (time_in_second % 3600) / 60;
  uint32_t seconds = time_in_second % 60;
  LED_BUF[7] = LED_CODE[hours / 10];
  LED_BUF[6] = LED_CODE[hours % 10];
  LED_BUF[5] = 0xBF;
  LED_BUF[4] = LED_CODE[minutes / 10];
  LED_BUF[3] = LED_CODE[minutes % 10];
  LED_BUF[2] = 0xBF;
  LED_BUF[1] = LED_CODE[seconds / 10];
  LED_BUF[0] = LED_CODE[seconds % 10];
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
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_buf[rx_len], 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* UART 协议：
   *   "Hhh"  -- 设置小时 (00-23)
   *   "Mmm"  -- 设置分钟 (00-59)
   *   "Sss"  -- 设置秒钟 (00-59)
   *   "Thhmmss" -- 同时设置时分秒
   *   每条指令以 \r 或 \n 结束
   */
  while (1)
  {
    if (rx_ready) {
      rx_ready = 0;

      if (rx_frame_len >= 1) {
        char cmd = rx_buf[0];
        uint32_t hours = (time_in_second / 3600) % 24;
        uint32_t minutes = (time_in_second % 3600) / 60;
        uint32_t seconds = time_in_second % 60;

        if (cmd == 'H' && rx_frame_len >= 3) {
          uint8_t hh = (rx_buf[1]-'0')*10 + (rx_buf[2]-'0');
          if (hh < 24) {
            hours = hh;
            time_in_second = hours*3600 + minutes*60 + seconds;
            refresh_led_from_time();
            HAL_UART_Transmit(&huart2, (uint8_t*)"OK\r\n", 4, HAL_MAX_DELAY);
          } else {
            HAL_UART_Transmit(&huart2, (uint8_t*)"ERR\r\n", 5, HAL_MAX_DELAY);
          }
        } else if (cmd == 'M' && rx_frame_len >= 3) {
          uint8_t mm = (rx_buf[1]-'0')*10 + (rx_buf[2]-'0');
          if (mm < 60) {
            minutes = mm;
            time_in_second = hours*3600 + minutes*60 + seconds;
            refresh_led_from_time();
            HAL_UART_Transmit(&huart2, (uint8_t*)"OK\r\n", 4, HAL_MAX_DELAY);
          } else {
            HAL_UART_Transmit(&huart2, (uint8_t*)"ERR\r\n", 5, HAL_MAX_DELAY);
          }
        } else if (cmd == 'S' && rx_frame_len >= 3) {
          uint8_t ss = (rx_buf[1]-'0')*10 + (rx_buf[2]-'0');
          if (ss < 60) {
            seconds = ss;
            time_in_second = hours*3600 + minutes*60 + seconds;
            refresh_led_from_time();
            HAL_UART_Transmit(&huart2, (uint8_t*)"OK\r\n", 4, HAL_MAX_DELAY);
          } else {
            HAL_UART_Transmit(&huart2, (uint8_t*)"ERR\r\n", 5, HAL_MAX_DELAY);
          }
        } else if (cmd == 'T' && rx_frame_len >= 7) {
          uint8_t hh = (rx_buf[1]-'0')*10 + (rx_buf[2]-'0');
          uint8_t mm = (rx_buf[3]-'0')*10 + (rx_buf[4]-'0');
          uint8_t ss = (rx_buf[5]-'0')*10 + (rx_buf[6]-'0');
          if (hh < 24 && mm < 60 && ss < 60) {
            time_in_second = hh*3600 + mm*60 + ss;
            refresh_led_from_time();
            HAL_UART_Transmit(&huart2, (uint8_t*)"OK\r\n", 4, HAL_MAX_DELAY);
          } else {
            HAL_UART_Transmit(&huart2, (uint8_t*)"ERR\r\n", 5, HAL_MAX_DELAY);
          }
        } else {
          HAL_UART_Transmit(&huart2, (uint8_t*)"?\r\n", 3, HAL_MAX_DELAY);
        }
      }
    }
    if (tx_time_flag) {
      tx_time_flag = 0;
      char tbuf[16];
      uint32_t hours = (time_in_second/3600) % 24;
      uint32_t minutes = (time_in_second % 3600) / 60;
      uint32_t seconds = time_in_second % 60;
      int len = sprintf(tbuf, "%02u:%02u:%02u\r\n",
                        (unsigned int)hours,
                        (unsigned int)minutes,
                        (unsigned int)seconds);
      HAL_UART_Transmit(&huart2, (uint8_t*)tbuf, len, HAL_MAX_DELAY);
    }
    __WFI();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 15999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1599;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SCLK_Pin|DIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SCLK_Pin DIO_Pin */
  GPIO_InitStruct.Pin = SCLK_Pin|DIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RCLK_Pin */
  GPIO_InitStruct.Pin = RCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7) {

    HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin, GPIO_PIN_RESET);

    out_595(LED_BUF[disp_ind]);
    out_595(1 << disp_ind);

    HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin, GPIO_PIN_SET);
    disp_ind = (disp_ind + 1) & 0x07;
  }
  if (htim->Instance == TIM6)
  {
    time_in_second = (time_in_second + 1) % 86400;
    refresh_led_from_time();
    tx_time_flag = 1;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint8_t ch = rx_buf[rx_len];          /* 当前收到的字节 */

    /* 帧结束：\n 或 \r */
    if (ch=='\r' || ch=='\n') {
      if (rx_len) {               /* 已经收了一些数据 */
        rx_frame_len = rx_len;  /* <<< 新增：保存有效字节数 */
        rx_ready     = 1;       /* 通知主循环 */
      }
      rx_len = 0;                 /* 重新开始下一帧 */
    }
    else if (rx_len < RX_BUF_LEN-1) {
      rx_len++;                         /* 推进写指针 */
    }                                     /* 溢出字节直接丢弃 */

    /* 继续收下一个字节 */
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_buf[rx_len], 1);
  }
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
