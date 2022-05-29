/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart_ring.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NS 128;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t Wave_LUT[128] = {
	2048, 2149, 2250, 2350, 2450, 2549, 2646, 2742, 2837, 2929, 3020, 3108, 3193, 3275, 3355, 3431,
	3504, 3574, 3639, 3701, 3759, 3812, 3861, 3906, 3946, 3982, 4013, 4039, 4060, 4076, 4087, 4094,
	4095, 4091, 4082, 4069, 4050, 4026, 3998, 3965, 3927, 3884, 3837, 3786, 3730, 3671, 3607, 3539,
	3468, 3394, 3316, 3235, 3151, 3064, 2975, 2883, 2790, 2695, 2598, 2500, 2400, 2300, 2199, 2098,
	1997, 1896, 1795, 1695, 1595, 1497, 1400, 1305, 1212, 1120, 1031, 944, 860, 779, 701, 627,
	556, 488, 424, 365, 309, 258, 211, 168, 130, 97, 69, 45, 26, 13, 4, 0,
	1, 8, 19, 35, 56, 82, 113, 149, 189, 234, 283, 336, 394, 456, 521, 591,
	664, 740, 820, 902, 987, 1075, 1166, 1258, 1353, 1449, 1546, 1645, 1745, 1845, 1946, 2047
};
uint8_t period = 40;
uint32_t counter = 0;
uint16_t i = 0;
/*uint8_t serial[16] = {1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0};*/
uint8_t serial[16] = {0,};
char a, b, c;
uint8_t buf[10] = {1,1,1,1,1,1,1,1,1,1};
uint8_t str[32];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void DWT_Init(void)
{
    SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
    DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;   // запускаем счётчик
}

void delay_micros(uint32_t us)
{
    uint32_t us_count_tic =  us * (SystemCoreClock / 1000000); // получаем кол-во тактов за 1 мкс и умножаем на наше значение
    DWT->CYCCNT = 0U; // обнуляем счётчик
    while(DWT->CYCCNT < us_count_tic);
}
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
  MX_DMA_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /*HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  DWT_Init();*/
 HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)Wave_LUT, 128, DAC_ALIGN_12B_R);
 HAL_TIM_Base_Start(&htim2);

//  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
  //	 	                        HAL_TIM_Base_Stop(&htim2);

  __HAL_UART_ENABLE_IT(&MYUART, UART_IT_RXNE); // включить прерывания usart'

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 if(uart_available()) // есть ли что-то в приёмном буфере, тогда читаем
	            {
	                    char str[SIZE_BF] = {0,};
	                    uint8_t i = 0;

	                    while(uart_available())
	                    {
	                            str[i++] = uart_read(); // читаем байт

	                            if(i == SIZE_BF - 1)
	                            {
	                                   // str[i] = '\0';
	                                    break;
	                            }

	                            HAL_Delay(2);
	                    }

	                   // str[i] = '\0';
	                   a = str[0];
	                   b = str[1];
                       c = str[2];

	                    if((a =='i') && (b =='d'))
	                    		{
	                    HAL_UART_Transmit(&MYUART, (uint8_t*)str, strlen(str), 100); // отправляем обратно что получили

	                    switch(c)
	                    {
	                    case 'a':
	                 buf[0] = 0;//стартовый бит
	                 buf[1] = 0;
	                 buf[2] = 1;
	                 buf[3] = 1;
	                 buf[4] = 0;
	                 buf[5] = 0;
	                 buf[6] = 0;
	                 buf[7] = 0;
	                 buf[8] = 1;
	                 buf[9] = 1;//стоповый бит

	                    break;
	                    case 'b':
	                // buf[8] = {0,1,1,0,0,0,1,0};
	                 buf[0] = 0; //стартовый бит
	                 buf[1] = 0;
	                 buf[2] = 1;
	                 buf[3] = 1;
	                 buf[4] = 0;
	                 buf[5] = 0;
	                 buf[6] = 0;
	                 buf[7] = 1;
	                 buf[8] = 0;
	                 buf[9] = 1;//стоповый бит

	                  	break;
	                    case 'c':
	                    // buf[8] = {0,1,1,0,0,0,1,1};
	                  buf[0] = 0;  //стартовый бит
	                  buf[1] = 0;
	                  buf[2] = 1;
	                  buf[3] = 1;
	                  buf[4] = 0;
	                  buf[5] = 0;
	                  buf[6] = 0;
	                  buf[7] = 1;
	                  buf[8] = 1;
	                  buf[9] = 1;//стоповый бит
	                   	 break;
	                    case 's':
	                  // buf[8] = {0,1,1,0,0,0,1,1};
	                  buf[0] = 0; //стартовый бит
	                  buf[1] = 0;
	                  buf[2] = 0;
	                  buf[3] = 0;
	                  buf[4] = 0;
	                  buf[5] = 0;
	                  buf[6] = 0;
	                  buf[7] = 0;
	                  buf[8] = 0;
	                  buf[9] = 1;//стоповый бит
	                  break;

	                    }///switch
	                    }////if id
	            }///if available


	      for(uint8_t i = 0; i < 10; i++)
	 	                   {
	 	                    if(buf[i] == 0)
	 	                    {

	 	                    	HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
	 	                        HAL_TIM_Base_Stop(&htim2);
	 	                        HAL_Delay(1);
	 	                    }
	 	                    else
	 	                    {
	 	                       HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)Wave_LUT, 128, DAC_ALIGN_12B_R);
	 	                       HAL_TIM_Base_Start(&htim2);
	 	                      HAL_Delay(1);
	 	                    }//
	 	                   } //for
	                                   HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)Wave_LUT, 128, DAC_ALIGN_12B_R);
	     	 	                       HAL_TIM_Base_Start(&htim2);

	     	 	                       HAL_Delay(10);


	 	              //     if (counter == 12800-1)
	 	                ///  	  {
	 	                  	//	  i++;
	 	                  		//  counter = 0;
	 	                  	//  }
	 	                  	 // if (i == 7)
	 	                  	 // {
	 	                  		//  i = 0;
	 	                  	  //}
	 	                  	 // counter++;





	 	                  //  }////for



	//  HAL_UART_Receive_IT(&huart2, (uint8_t*)serial, 15);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)serial, 15, 100);
	  /*if (serial[i] == 0)
	  {
		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
		  htim2.Instance->CR1 &= ~TIM_CR1_CEN;
	  }
	  else
	  {
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)Wave_LUT, 128, DAC_ALIGN_12B_R);
		  htim2.Instance->CR1 |= TIM_CR1_CEN;
	  }

	  if (counter == 12800-1)
	  {
		  i++;
		  counter = 0;
	  }
	  if (i == 15)
	  {
		  i = 0;
	  }
	  counter++;*/

	 if (HAL_GPIO_ReadPin(LO_GPIO_Port,LO_Pin)==0) //Нажатие кнопки понижения частоты
	 {
		  period++;
		 TIM2->CNT = 0;
		  __HAL_TIM_SET_AUTORELOAD(&htim2, period);
		  HAL_Delay(200);
		  snprintf(str, 32, "Period %d\n", period);
		  HAL_UART_Transmit(&huart2, (uint8_t*)str, 32, 100);
	  }
	  if (HAL_GPIO_ReadPin(HI_GPIO_Port,HI_Pin)==0) //Нажатие кнопки повышения частоты
	  {
		  period--;
		  TIM2->CNT = 0;
		  __HAL_TIM_SET_AUTORELOAD(&htim2, period);
		  HAL_Delay(200);
		  snprintf(str, 32, "Period %d\n", period);
		  HAL_UART_Transmit(&huart2, (uint8_t*)str, 32, 100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 39;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pins : HI_Pin SWITCH_Pin */
  GPIO_InitStruct.Pin = HI_Pin|SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LO_Pin */
  GPIO_InitStruct.Pin = LO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LO_GPIO_Port, &GPIO_InitStruct);

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

