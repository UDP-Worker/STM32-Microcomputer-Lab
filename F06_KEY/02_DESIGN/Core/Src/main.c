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
GPIO_TypeDef* row_port[4] = {L1_GPIO_Port, L2_GPIO_Port, L3_GPIO_Port, L4_GPIO_Port};
uint16_t     row_pin [4] = {L1_Pin,      L2_Pin,      L3_Pin,      L4_Pin     };

GPIO_TypeDef* col_port[4] = {R1_GPIO_Port, R2_GPIO_Port, R3_GPIO_Port, R4_GPIO_Port};
uint16_t     col_pin [4] = {R1_Pin,      R2_Pin,      R3_Pin,      R4_Pin     };

#define MAX_SID_LEN 10
const uint8_t LED_CODE[] = {
  0xF9, // 1
  0xA4, // 2
  0xB0, // 3
  0x99, // 4
  0x92, // 5
  0x82, // 6
  0xF8, // 7
  0x80, // 8
  0x90, // 9
  0xFF, // *
  0xC0, // 0
  0xFF, // #
};
uint8_t LED_BUF[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t disp_ind = 0;
uint8_t cur_row=0, cur_col, key_value;

uint8_t sid_buf[MAX_SID_LEN];
uint8_t sid_len=0, sid_scroll_len=0, scroll_idx=0;
int8_t  scroll_dir=1;
uint8_t input_mode=0;



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
  __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
  __HAL_TIM_DISABLE(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 行拉高，逐行扫描 */
        for (int i = 0; i < 4; i++)
            HAL_GPIO_WritePin(row_port[i], row_pin[i], GPIO_PIN_SET);
        HAL_GPIO_WritePin(row_port[cur_row], row_pin[cur_row], GPIO_PIN_RESET);

        /* 只扫描前三列 */
        for (cur_col = 0; cur_col < 3; cur_col++) {
            if (HAL_GPIO_ReadPin(col_port[cur_col], col_pin[cur_col]) == GPIO_PIN_RESET) {
                HAL_Delay(20);
                if (HAL_GPIO_ReadPin(col_port[cur_col], col_pin[cur_col]) == GPIO_PIN_RESET) {
                    key_value = cur_row * 3 + cur_col;
                    /* # 键切换模�? */
                    if (key_value == 11) {
                        if (!input_mode) {
                            input_mode = 1; sid_len = 0; scroll_idx = 0;
                            for (int j = 0; j < 8; j++) LED_BUF[j] = 0xFF;
                            __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
                            __HAL_TIM_DISABLE(&htim6);
                        } else {
                            input_mode = 0;
                            sid_scroll_len = sid_len + 2;
                            scroll_idx = 0;
                            __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
                            __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
                            __HAL_TIM_ENABLE(&htim6);
                        }
                    }
                    else if (input_mode) {
                        if ((key_value <= 8 || key_value == 10 ) && sid_len < MAX_SID_LEN)
                        {
                            sid_buf[sid_len++] = key_value;
                            //for (int j = 7; j > 0; j--) LED_BUF[j] = LED_BUF[j-1];
                            //LED_BUF[0] = LED_CODE[key_value];
                          for (int j = 0; j < 7; j++)
                            LED_BUF[j] = LED_BUF[j+1];
                          LED_BUF[7] = LED_CODE[key_value];
                        }
                    }
                    else {

                        if (key_value == 9) scroll_dir = -scroll_dir;
                    }
                    while (HAL_GPIO_ReadPin(col_port[cur_col], col_pin[cur_col]) == GPIO_PIN_RESET);
                    HAL_Delay(20);
                    break;
                }
            }
        }
        cur_row = (cur_row + 1) & 0x03;
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
  htim6.Init.Period = 499;
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
    /* 刷新动�?�数码管 */
    HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin, GPIO_PIN_RESET);
    out_595(LED_BUF[disp_ind]);
    out_595(1 << disp_ind);
    HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin, GPIO_PIN_SET);
    disp_ind = (disp_ind + 1) & 0x07;
  }
  else if (htim->Instance == TIM6) {
    /* 滚动学号 */
    uint8_t idx;
    for (int i = 0; i < 8; i++) {
      idx = (scroll_idx + i) % sid_scroll_len;
      if (idx < sid_len) LED_BUF[i] = LED_CODE[sid_buf[idx]];
      else               LED_BUF[i] = 0xFF;
    }
    scroll_idx = (scroll_idx + scroll_dir + sid_scroll_len) % sid_scroll_len;
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
