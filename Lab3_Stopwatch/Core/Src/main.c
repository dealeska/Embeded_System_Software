/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const uint8_t simple_digits[]   = { 0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xD8, 0x80, 0x90 };
                            /*  0.    1.    2.    3.    4.    5.    6.    7.    8.    9.  */
const uint8_t digits_with_dot[] = { 0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x58, 0x00, 0x10 }; 
                      /*   1     2     3     4  */
const uint8_t positions[] = { 0x08, 0x04, 0x02, 0x01 };

uint8_t is_stopwatch_running;
uint8_t is_pause;
volatile uint16_t stopwatch_time;
volatile uint8_t pos_to_draw;

volatile uint8_t flag_irq_btn_start_stop;
volatile uint32_t time_irq_btn_start_stop;
volatile uint8_t flag_irq_btn_pause;
volatile uint32_t time_irq_btn_pause;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void write_digit(uint8_t pos, uint8_t digit);
void set_led(GPIO_TypeDef* port, uint8_t pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void write_digit(uint8_t pos, uint8_t digit) {
  int j = 0;
	while(j < 8) {
	  if (((digit << j) & 0x80) != 0) {
		  HAL_GPIO_WritePin(DS_DATA_GPIO_Port, DS_DATA_Pin, GPIO_PIN_SET);
		} else {
		  HAL_GPIO_WritePin(DS_DATA_GPIO_Port, DS_DATA_Pin, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(DS_SHIFT_GPIO_Port, DS_SHIFT_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DS_SHIFT_GPIO_Port, DS_SHIFT_Pin, GPIO_PIN_RESET);
		j++;
	}
	
	j = 0;
	while(j < 8) {
	  if (((pos << j) & 0x80) != 0) {
		  HAL_GPIO_WritePin(DS_DATA_GPIO_Port, DS_DATA_Pin, GPIO_PIN_SET);
		} else {
		  HAL_GPIO_WritePin(DS_DATA_GPIO_Port, DS_DATA_Pin, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(DS_SHIFT_GPIO_Port, DS_SHIFT_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DS_SHIFT_GPIO_Port, DS_SHIFT_Pin, GPIO_PIN_RESET);
		j++;
	}
	
	HAL_GPIO_WritePin(DS_LATCH_GPIO_Port, DS_LATCH_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DS_LATCH_GPIO_Port, DS_LATCH_Pin, GPIO_PIN_RESET);
}

void set_led(GPIO_TypeDef* port, uint8_t pin) {
  HAL_GPIO_WritePin(LED_D13_GPIO_Port, LED_D13_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_D12_GPIO_Port, LED_D12_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_D11_GPIO_Port, LED_D11_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if(htim->Instance == TIM1) {
		 if (stopwatch_time < 9999) {
		   stopwatch_time++;
		 } else {
		   stopwatch_time = 0;
		 }
	 }
	 
	 if(htim->Instance == TIM2) {
	   switch (pos_to_draw) {
		   case 0: 
				 write_digit(positions[0], simple_digits[stopwatch_time % 10]);
			   break;
			 case 1:
				 write_digit(positions[1], digits_with_dot[stopwatch_time / 10 % 10]);
			   break;
			 case 2:
				 if (((stopwatch_time / 100 % 10) != 0) || ((stopwatch_time / 1000) != 0)) {
					 write_digit(positions[2], simple_digits[stopwatch_time / 100 % 10]);
				 }
			   break;
			 case 3:
				 if ((stopwatch_time / 1000) != 0) {
					 write_digit(positions[3], simple_digits[stopwatch_time / 1000]);
				 }
			   break;
		 }
		 
		 if(pos_to_draw == 3) {
		   pos_to_draw = 0;
		 } else {
		   pos_to_draw++;
		 }
	 }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == BTN_START_STOP_Pin) {
		HAL_NVIC_DisableIRQ(EXTI1_IRQn);
		flag_irq_btn_start_stop = 1;
    time_irq_btn_start_stop = HAL_GetTick();
		
    if(!is_stopwatch_running) {
		  is_stopwatch_running = 1;
			HAL_TIM_Base_Start_IT(&htim1);
			set_led(LED_D12_GPIO_Port, LED_D12_Pin);
		} else if(is_stopwatch_running || is_pause) {
		  is_stopwatch_running = 0;
			is_pause = 0;
			HAL_TIM_Base_Stop_IT(&htim1);
			stopwatch_time = 0;
			set_led(LED_D13_GPIO_Port, LED_D13_Pin);
		}
  }
	
	if(GPIO_Pin == BTN_PAUSE_Pin) {
		HAL_NVIC_DisableIRQ(EXTI4_IRQn);
		flag_irq_btn_pause = 1;
    time_irq_btn_pause = HAL_GetTick();
		
    if(is_stopwatch_running && !is_pause) {
			is_pause = 1;
			HAL_TIM_Base_Stop_IT(&htim1);
			set_led(LED_D11_GPIO_Port, LED_D11_Pin);
		} else if (is_stopwatch_running && is_pause) {
		  is_pause = 0;
			HAL_TIM_Base_Start_IT(&htim1);
			set_led(LED_D12_GPIO_Port, LED_D12_Pin);
		}
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(LED_D13_GPIO_Port, LED_D13_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_D12_GPIO_Port, LED_D12_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_D11_GPIO_Port, LED_D11_Pin, GPIO_PIN_SET);
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(flag_irq_btn_start_stop && (HAL_GetTick() - time_irq_btn_start_stop) > 300) {
      __HAL_GPIO_EXTI_CLEAR_IT(BTN_START_STOP_Pin);
      NVIC_ClearPendingIRQ(EXTI1_IRQn);
      HAL_NVIC_EnableIRQ(EXTI1_IRQn);
      flag_irq_btn_start_stop = 0;
    }
		
		if(flag_irq_btn_pause && (HAL_GetTick() - time_irq_btn_pause) > 300) {
      __HAL_GPIO_EXTI_CLEAR_IT(BTN_PAUSE_Pin);
      NVIC_ClearPendingIRQ(EXTI4_IRQn);
      HAL_NVIC_EnableIRQ(EXTI4_IRQn);
      flag_irq_btn_pause = 0;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_D13_Pin|LED_D12_Pin|LED_D11_Pin|DS_SHIFT_Pin
                          |DS_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS_LATCH_GPIO_Port, DS_LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_START_STOP_Pin BTN_PAUSE_Pin */
  GPIO_InitStruct.Pin = BTN_START_STOP_Pin|BTN_PAUSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_D13_Pin LED_D12_Pin LED_D11_Pin DS_SHIFT_Pin
                           DS_DATA_Pin */
  GPIO_InitStruct.Pin = LED_D13_Pin|LED_D12_Pin|LED_D11_Pin|DS_SHIFT_Pin
                          |DS_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DS_LATCH_Pin */
  GPIO_InitStruct.Pin = DS_LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS_LATCH_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
