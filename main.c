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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "string.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 100
char ip_buffer[BUFFER_SIZE];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define STEPS_PER_REV 3200
char command;

void delay_us(uint32_t us) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // Reset TIM2 counter
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // Wait for the timer to reach the input value
}

void stepperMove(bool direction, uint16_t steps) {
	// Set direction pin (PA0)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, direction);

    for (uint16_t i = 0; i < steps; i++) {
        // Generate step pulse
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // Step pin high
        delay_us(2000);  // Delay for 1 ms (adjust for timing)

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // Step pin low
        delay_us(2000);  // Delay for 1 ms (adjust for timing)
    }
}


void stepperMoveShoot(bool direction, uint16_t steps) {
	// Set direction pin (PA0)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, direction);

    for (uint16_t i = 0; i < steps; i++) {
        // Generate step pulse
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);  // Step pin high
        delay_us(2000);  // Delay for 1 ms (adjust for timing)

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);  // Step pin low
        delay_us(2000);  // Delay for 1 ms (adjust for timing)
    }
}


void homeAndCenterStepper() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // Enable motor driver (EN LOW)
    uint16_t stepCount = 0;

    // Move left until left limit switch is pressed, count steps
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) != GPIO_PIN_RESET) {
        stepperMove(true, 5);  // Move left
        stepCount+=5;
    }

    // Move right until right limit switch is pressed, count steps
    uint16_t totalSteps = 0;
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) != GPIO_PIN_RESET) {
        stepperMove(false, 5);  // Move right
        totalSteps+=5;
    }

    // Move back to the middle
    uint16_t stepsToCenter = totalSteps / 2;
    stepperMove(true, stepsToCenter);  // Move back to the middle

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // Disable motor driver (EN HIGH)
}

bool Receive_IP_Address(char *buffer, uint16_t buffer_size)
{
  uint16_t index = 0;
  uint8_t rx_byte;
  uint32_t start_time = HAL_GetTick();
  uint32_t timeout = 5000; // Timeout in milliseconds

  // Receive the IP address (assumed to be terminated with '\n')
  while (HAL_GetTick() - start_time < timeout && index < buffer_size - 1)
  {
    if (HAL_UART_Receive(&huart1, &rx_byte, 1, 100) == HAL_OK)
    {
      if (rx_byte == '\n')
      {
        buffer[index] = '\0'; // Null-terminate the string
        return true;
      }
      else if (rx_byte != '\r')
      {
        buffer[index++] = rx_byte;
      }
    }
  }

  // If the IP address was not fully received, return false
  return false;
}


void getAndSendIP() {
	  command = 'R';
	  HAL_UART_Transmit(&huart1, (uint8_t *)&command, 1, HAL_MAX_DELAY);

	  // Wait for response from ESP32-CAM
	  memset(ip_buffer, 0, BUFFER_SIZE); // Clear the buffer

	  // Receive the IP address
	  if (Receive_IP_Address(ip_buffer, BUFFER_SIZE))
	  {
		// Send the IP address to the laptop via USART2
		HAL_UART_Transmit(&huart2, (uint8_t *)ip_buffer, strlen(ip_buffer), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
	  }
	  else
	  {
		// Handle error (e.g., timeout)
		HAL_UART_Transmit(&huart2, (uint8_t *)"Failed to receive IP address.\r\n", 31, HAL_MAX_DELAY);
	  }
}

int getAngleFromPC(void)
{
  char rx_buffer[100];  // Buffer to store received data
  uint8_t index = 0;
  uint8_t rx_byte;
  int angle = 0;
  uint32_t start_time = HAL_GetTick();
  uint32_t timeout = 10000;  // 10 seconds timeout

  // Receive data until newline or timeout
  while ((HAL_GetTick() - start_time) < timeout)
  {
    if (HAL_UART_Receive(&huart2, &rx_byte, 1, 100) == HAL_OK)
    {
      if (rx_byte == '\r' || rx_byte == '\n')
      {
        if (index > 0) // Only process if we have received some data
        {
          rx_buffer[index] = '\0';  // Null-terminate the string

          // Parse the angle
          angle = atoi(rx_buffer);

          // Break out of the loop since we have received the angle
          return angle;
        }
        else
        {
          // Ignore empty lines
          continue;
        }
      }
      else
      {
        if (index < sizeof(rx_buffer) - 1)
        {
          rx_buffer[index++] = rx_byte;
        }
        else
        {
          // Buffer overflow
          char *error_msg = "Input too long. Please enter a shorter value:\r\n";
          HAL_UART_Transmit(&huart2, (uint8_t *)error_msg, strlen(error_msg), HAL_MAX_DELAY);
          // Reset index and buffer
          index = 0;
          memset(rx_buffer, 0, sizeof(rx_buffer));
          start_time = HAL_GetTick();  // Reset timeout
        }
      }
    }
  }

  // Timeout occurred
  char *timeout_msg = "Timeout occurred. Using default angle 1.\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)timeout_msg, strlen(timeout_msg), HAL_MAX_DELAY);
  return 1;  // Default angle
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)  // Button is pressed
	  {
		  // HOME
		  HAL_Delay(500);
		  char commandh[] = "H\n";
		  HAL_UART_Transmit(&huart3, (uint8_t *)commandh, sizeof(commandh) - 1, HAL_MAX_DELAY);
		  HAL_Delay(6000);

		  // GET & SEND IP TO COMPUTER
		  getAndSendIP();

		  // RECIEVE ANGLE
		  int angle = getAngleFromPC();

		  // SEND ANGLE
		  char angle_str[12];
		  sprintf(angle_str, "%d\n", angle);
		  HAL_UART_Transmit(&huart3, (uint8_t *)angle_str, strlen(angle_str), HAL_MAX_DELAY);
		  HAL_Delay(4000);

		  // SHOOT
		  char commands[] = "S\n";
		  HAL_UART_Transmit(&huart3, (uint8_t *)commands, sizeof(commands) - 1, HAL_MAX_DELAY);
		  HAL_Delay(5000);
	  }

	  HAL_Delay(100);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
