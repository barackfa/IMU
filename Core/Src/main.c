/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "IMU.h"
#include <string.h>
#include <stdio.h>

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
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// IMU address
uint16_t IMU_address = 214;
uint8_t IMU_Data_Ready;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
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
	uint8_t buf[20];
	buf[0] = 1;
	buf[1] = 12;
	uint8_t buff[3];
	uint8_t  OUTX_L_XL;
	uint8_t  OUTX_H_XL;
	uint8_t  OUTY_L_XL;
	uint8_t  OUTY_H_XL;
	uint8_t  OUTZ_L_XL;
	uint8_t  OUTZ_H_XL;

	uint8_t  OUTX_L_G;
	uint8_t  OUTX_H_G;
	uint8_t  OUTY_L_G;
	uint8_t  OUTY_H_G;
	uint8_t  OUTZ_L_G;
	uint8_t  OUTZ_H_G;

	uint8_t IMU_Data_Ready;
	uint8_t XL_Mode = 0x60;
	uint8_t G_Mode =  0xA0;
	uint8_t XL_IT = 0x01;

	uint16_t XL_X;
	uint16_t XL_Y;
	uint16_t XL_Z;

	uint16_t G_X;
	uint16_t G_Y;
	uint16_t G_Z;
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
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  IMU_init();
//	  HAL_I2C_Mem_Write  ( &hi2c3 ,IMU_address, 0x10, 1, &XL_Mode, 1, 10000); // 0x10 = CTRL1_XL
//	  	  HAL_I2C_Mem_Write  ( &hi2c3 ,IMU_address, 0x0D, 1, &XL_IT, 1, 10000); // INT1_CTRL (0Dh)
//	  	  //HAL_Delay(10);
//	  	  //HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x10, 1, buf[1], 1, 10000);
	  	  HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x0D, 1, buf, 1, 10000);
	  	  HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x1E, 1, &IMU_Data_Ready, 1, 10000); //STATUS_REG (1Eh)

	  	  if((IMU_Data_Ready & 0x01) == 1){
	  		  HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x28, 1, &OUTX_L_XL, 1, 10000); // OUTX_L_XL (28h)
	  		  HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x29, 1, &OUTX_H_XL, 1, 10000); // OUTX_H_XL (29h)
	  		  HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x2A, 1, &OUTY_L_XL, 1, 10000); // OUTY_L_XL (2Ah)
	  		  HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x2B, 1, &OUTY_H_XL, 1, 10000); // OUTY_H_XL (2Bh)
	  		  HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x2C, 1, &OUTZ_L_XL, 1, 10000); // OUTZ_L_XL (2Ch)
	  		  HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x2D, 1, &OUTZ_H_XL, 1, 10000); // OUTZ_H_XL (2Dh)
	  		  XL_X = (OUTX_H_XL << 8) + OUTX_L_XL;
	  		  XL_Y = (OUTY_H_XL << 8) + OUTY_L_XL;
	  		  XL_Z = (OUTZ_H_XL << 8) + OUTZ_L_XL;
	  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  		  HAL_Delay(100);
	  		  sprintf((char*)buff, "%d ",OUTX_H_XL);
	  		  HAL_UART_Transmit  ( &huart2, buff, strlen((char*)buff), 10000);
	  		  HAL_Delay(5);
	  		  sprintf((char*)buff, "%d ",OUTX_L_XL);
	  		  HAL_UART_Transmit  ( &huart2, buff, strlen((char*)buff), 10000);
	  		  HAL_Delay(5);
	  	  }

	  	if((IMU_Data_Ready/2) > 0){
	  		  		  HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x22, 1, &OUTX_L_G, 1, 10000); // xl
	  		  		  HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x23, 1, &OUTX_H_G, 1, 10000); // xh
	  		  		  HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x24, 1, &OUTY_L_G, 1, 10000); // yl
	  		  		  HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x25, 1, &OUTY_H_G, 1, 10000); // yh
	  		  		  HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x26, 1, &OUTZ_L_G, 1, 10000); // zl
	  		  		  HAL_I2C_Mem_Read  ( &hi2c3 ,IMU_address, 0x27, 1, &OUTZ_H_G, 1, 10000); // zh
	  		  		  G_X = ((uint16_t)OUTX_H_G << 8) + OUTX_L_G;
	  		  		  G_Y = ((uint16_t)OUTY_H_G << 8) + OUTY_L_G;
	  		  		  G_Z = ((uint16_t)OUTZ_H_G << 8) + OUTZ_L_G;
	  		  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  		  		  HAL_Delay(100);
	  		  		  sprintf((char*)buff, "GZ: %d ",OUTZ_H_G);
	  		  		  HAL_UART_Transmit  ( &huart2, buff, strlen((char*)buff), 10000);
	  		  		  HAL_Delay(5);
	  		  		  sprintf((char*)buff, "%d\n\r",OUTZ_L_G);
	  		  		  HAL_UART_Transmit  ( &huart2, buff, strlen((char*)buff), 10000);
	  		  		  HAL_Delay(5);
	  		  	  }
//	  XL_X = get_ACC_X();
//	  //printf("%d",XL_X);
//	  sprintf((char*)buff, "%d ",XL_X);
//	  HAL_UART_Transmit  ( &huart2, buff, strlen((char*)buff), 10000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
