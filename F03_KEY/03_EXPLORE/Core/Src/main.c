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

/* USER CODE BEGIN PV */
/* line IO pin and port */
GPIO_TypeDef * row_port[4] = {L1_GPIO_Port,L2_GPIO_Port,L3_GPIO_Port,L4_GPIO_Port};
uint32_t row_pin[4] = {L1_Pin,L2_Pin,L3_Pin,L4_Pin};

/* column IO pin and port */
GPIO_TypeDef * col_port[4] = {R1_GPIO_Port,R2_GPIO_Port,R3_GPIO_Port,R4_GPIO_Port};
uint32_t col_pin[4] = {R1_Pin,R2_Pin,R3_Pin,R4_Pin};

const uint8_t LED_CODE[] = {
  0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,
  0x80,0x90,0x88,0x83,0xC6,0xA1,0x86,0x8E,
  0xBF
};
uint32_t time_in_second = 0;
uint8_t LED_BUF[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t disp_ind = 0;
uint8_t status = 0; //0 for regular clock, 1 for editing clock.
uint8_t edit_pos = 0;              // 当前在 0~5 段上编辑（0→H 高位，1→H 低位，2→M 高位…5→S 低位）
const uint8_t edit_idx[6] = {7,6,4,3,1,0};
const uint32_t weight[6] = {36000, 3600, 600, 60, 10, 1};
const uint8_t max_digit[6] = {3,10,6,10,6,10};
uint16_t blink_cnt = 0;            // TIM7 毫秒计数，用于 500 ms 闪烁
uint8_t  blink_on  = 1;            // 当前是否“显示”编辑位

uint8_t cur_row = 0, cur_col = 0, key_value = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
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

void judge_and_retrieve(int key_value)
{
  uint32_t time_in_digits[6];
  uint32_t hours = (time_in_second/3600) % 24;
  uint32_t minutes = (time_in_second - (hours*3600)) / 60;
  uint32_t seconds = time_in_second % 60;
  time_in_digits[0] = hours / 10;
  time_in_digits[1] = hours % 10;
  time_in_digits[2] = minutes / 10;
  time_in_digits[3] = minutes % 10;
  time_in_digits[4] = seconds / 10;
  time_in_digits[5] = seconds % 10;
  if (key_value == 14)
  {
    HAL_TIM_Base_Start_IT(&htim6);
    status = 0;
  }
  if (key_value == 12)
  {
    edit_pos = (edit_pos + 1) % 6;
  }
  if (key_value == 0)
  {
    time_in_second = time_in_second + weight[edit_pos] * (1 % max_digit[edit_pos] - time_in_digits[edit_pos]);
  }
  if (key_value == 1)
  {
    time_in_second = time_in_second + weight[edit_pos] * (2 % max_digit[edit_pos] - time_in_digits[edit_pos]);
  }
  if (key_value == 2)
  {
    time_in_second = time_in_second + weight[edit_pos] * (3 % max_digit[edit_pos] - time_in_digits[edit_pos]);
  }
  if (key_value == 4)
  {
    time_in_second = time_in_second + weight[edit_pos] * (4 % max_digit[edit_pos] - time_in_digits[edit_pos]);
  }
  if (key_value == 5)
  {
    time_in_second = time_in_second + weight[edit_pos] * (5 % max_digit[edit_pos] - time_in_digits[edit_pos]);
  }
  if (key_value == 6)
  {
    time_in_second = time_in_second + weight[edit_pos] * (6 % max_digit[edit_pos] - time_in_digits[edit_pos]);
  }
  if (key_value == 8)
  {
    time_in_second = time_in_second + weight[edit_pos] * (7 % max_digit[edit_pos] - time_in_digits[edit_pos]);
  }
  if (key_value == 9)
  {
    time_in_second = time_in_second + weight[edit_pos] * (8 % max_digit[edit_pos] - time_in_digits[edit_pos]);
  }
  if (key_value == 10)
  {
    time_in_second = time_in_second + weight[edit_pos] * (9 % max_digit[edit_pos] - time_in_digits[edit_pos]);
  }
  if (key_value == 13)
  {
    time_in_second = time_in_second + weight[edit_pos] * (0 - time_in_digits[edit_pos]);
  }
  time_in_second = time_in_second % 86400;
  hours = (time_in_second/3600) % 24;
  minutes = (time_in_second - (hours*3600)) / 60;
  seconds = time_in_second % 60;
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (status == 0)
    {
      for (int i = 0; i < 4; i++)
      {
        HAL_GPIO_WritePin(row_port[i],row_pin[i],GPIO_PIN_SET);
      }
      HAL_GPIO_WritePin(row_port[cur_row],row_pin[cur_row],GPIO_PIN_RESET);
      for (cur_col = 0; cur_col < 4; cur_col++)
      {
        if (HAL_GPIO_ReadPin(col_port[cur_col], col_pin[cur_col]) == GPIO_PIN_RESET)
        {
          HAL_Delay(20);

          if (HAL_GPIO_ReadPin(col_port[cur_col], col_pin[cur_col]) == GPIO_PIN_RESET)
          {
            key_value = cur_row * 4 + cur_col;
            if (key_value == 14)
            {
              HAL_TIM_Base_Stop_IT(&htim6);
              status = 1;
            }

            while (HAL_GPIO_ReadPin(col_port[cur_col], col_pin[cur_col]) == GPIO_PIN_RESET){}
            HAL_Delay(20);
            break;
          }
        }
      }
      cur_row = (cur_row + 1) & 0x03;
    }
    else
    {
      for (int i = 0; i < 4; i++)
      {
        HAL_GPIO_WritePin(row_port[i],row_pin[i],GPIO_PIN_SET);
      }
      HAL_GPIO_WritePin(row_port[cur_row],row_pin[cur_row],GPIO_PIN_RESET);
      for (cur_col = 0; cur_col < 4; cur_col++)
      {
        if (HAL_GPIO_ReadPin(col_port[cur_col], col_pin[cur_col]) == GPIO_PIN_RESET)
        {
          HAL_Delay(20);

          if (HAL_GPIO_ReadPin(col_port[cur_col], col_pin[cur_col]) == GPIO_PIN_RESET)
          {
            key_value = cur_row * 4 + cur_col;
            judge_and_retrieve(key_value);

            while (HAL_GPIO_ReadPin(col_port[cur_col], col_pin[cur_col]) == GPIO_PIN_RESET){}
            HAL_Delay(20);
            break;
          }
        }
      }
      cur_row = (cur_row + 1) & 0x03;
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
  HAL_GPIO_WritePin(GPIOA, SCLK_Pin|DIO_Pin|L3_Pin|L2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L4_Pin|RCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SCLK_Pin DIO_Pin L3_Pin L2_Pin */
  GPIO_InitStruct.Pin = SCLK_Pin|DIO_Pin|L3_Pin|L2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : L4_Pin RCLK_Pin */
  GPIO_InitStruct.Pin = L4_Pin|RCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : L1_Pin */
  GPIO_InitStruct.Pin = L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : R4_Pin */
  GPIO_InitStruct.Pin = R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(R4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R3_Pin R1_Pin R2_Pin */
  GPIO_InitStruct.Pin = R3_Pin|R1_Pin|R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7) {

    HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin, GPIO_PIN_RESET);

    uint8_t seg = LED_BUF[disp_ind];
    // 如果在编辑模式，且这个位正好是 edit_idx[edit_pos]，
    // 且 blink_on==0，就输出 0xFF（熄灭）来闪烁
    if (status == 1 && disp_ind == edit_idx[edit_pos] && blink_on == 0) {
      seg = 0xFF;
    }
    out_595(seg);
    out_595(1 << disp_ind);

    HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin, GPIO_PIN_SET);
    disp_ind = (disp_ind + 1) & 0x07;

    // —— 增加闪烁逻辑 ——
    if (++blink_cnt >= 500) {
      blink_cnt = 0;
      blink_on ^= 1;
    }
  }
  if (htim->Instance == TIM6)
  {
    time_in_second = (time_in_second + 1) % 86400;
    uint32_t hours = (time_in_second/3600) % 24;
    uint32_t minutes = (time_in_second - (hours*3600)) / 60;
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
