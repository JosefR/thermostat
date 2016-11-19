#include "stm32f0xx_hal.h"

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void uart_init();

/* HDC1008 */
#define HDC1008_I2C_ADDR 0x43
#define HDC1008_REG_TEMP 0x00
#define HDC1008_REG_HUM 0x01
#define HDC1008_REG_CFG 0x02
static uint8_t s_hdc1008_temp[2];
static uint8_t s_hdc1008_hum[2];
static void hdc1008_init();
static void hdc1008_read();

const char to_hex[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'a', 'b', 'c', 'd', 'e', 'f'};

int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();

  uart_init();

  hdc1008_init();

        char message[] = "t=xxxx, h=xxxx\n\r";
        char message2[] = "t=xx.x, h=xx.x\n\r";


    while (1) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(200);

        hdc1008_read();

        message[2] = to_hex[0xf & (s_hdc1008_temp[0] >> 4)];
        message[3] = to_hex[0xf & (s_hdc1008_temp[0])];
        message[4] = to_hex[0xf & (s_hdc1008_temp[1] >> 4)];
        message[5] = to_hex[0xf & s_hdc1008_temp[1]];
        message[10] = to_hex[0xf & (s_hdc1008_hum[0] >> 4)];
        message[11] = to_hex[0xf & (s_hdc1008_hum[0])];
        message[12] = to_hex[0xf & (s_hdc1008_hum[1] >> 4)];
        message[13] = to_hex[0xf & s_hdc1008_hum[1]];

        HAL_UART_Transmit(&huart2, (uint8_t*)message, 16, 5000);

        int32_t tmp = (s_hdc1008_temp[1] << 8 |  s_hdc1008_temp[0]) * 165;
        float temp = (float)tmp / (2e16) - 40;

/*        message2[5] = tmp % 10;
        message2[3] = (temp / 1000) % 10 + ('0' - 1);
        message2[2] = (temp / 10000) % 10 + ('0' - 1);*/
        HAL_UART_Transmit(&huart2, (uint8_t*)message2, 16, 5000);
    }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

}

static void uart_init()
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

static void hdc1008_init()
{
    uint8_t *data = { HDC1008_REG_CFG, 0x00, 0x00 };

    HAL_I2C_Master_Transmit(&hi2c1, HDC1008_I2C_ADDR << 1, data, 3, 5000);

}

static void hdc1008_read()
{
    uint8_t data_wr[1] = { 0 };

    data_wr[0] = HDC1008_REG_TEMP;
    HAL_I2C_Master_Transmit(&hi2c1, HDC1008_I2C_ADDR << 1, data_wr, 1, 5000);
    HAL_Delay(20);
    HAL_I2C_Master_Receive(&hi2c1, HDC1008_I2C_ADDR << 1, s_hdc1008_temp, 2, 5000);

    data_wr[0] = HDC1008_REG_HUM;
    HAL_I2C_Master_Transmit(&hi2c1, HDC1008_I2C_ADDR << 1, data_wr, 1, 5000);
    HAL_Delay(20);
    HAL_I2C_Master_Receive(&hi2c1, HDC1008_I2C_ADDR << 1, s_hdc1008_hum, 2, 5000);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  */

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif
