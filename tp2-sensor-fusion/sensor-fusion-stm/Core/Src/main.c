/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
//#include "stm32f3xx_hal_def.h"
#include "mpu6500.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

HAL_StatusTypeDef MPU_Init(I2C_HandleTypeDef * hi2c);

HAL_StatusTypeDef MPU_ReadData(I2C_HandleTypeDef * hi2c, MPU_data_t * d);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  MPU_Init(&hi2c1);

  while (1)
  {
    /* USER CODE END WHILE */
	 MPU_data_t d;
	 HAL_StatusTypeDef status = MPU_ReadData(&hi2c1, &d);
	 if (status == HAL_OK) {
		 int16_t dd[6];
		 dd[0] = (int16_t)(100.0*d.accel.x);
		 dd[1] = (int16_t)(100.0*d.accel.y);
		 dd[2] = (int16_t)(100.0*d.accel.z);
		 dd[3] = (int16_t)d.gyro.x;
		 dd[4] = (int16_t)d.gyro.y;
		 dd[5] = (int16_t)d.gyro.z;

		 unsigned char buffer[500];
		 int n = sprintf((char *)buffer, "Acc: X=%d, Y=%d, Z=%d --- Gyro: X=%d, Y=%d, Z=%d\r\n",
				 dd[0], dd[1], dd[2], dd[3], dd[4], dd[5]);
		 HAL_UART_Transmit(&huart2, buffer, n, 1000);
//		 for (unsigned int i = 0; i < 6; i++) {
//			 unsigned char buffer[100];
//			 int n = sprintf((char *)buffer, "%d", dd[i]);
//			 if (n >=0 &&  n < 100) {
//				 status = HAL_UART_Transmit(&huart2, buffer, n, 1000);
//				 if (status != HAL_OK) {
//					 break;
//				 }
//			 }
//		 }
//
		 HAL_Delay(1000); // wait one second
	 }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 38400;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

HAL_StatusTypeDef MPU_Init(I2C_HandleTypeDef * hi2c)
{
	unsigned char buffer[1];


	// check if mpu is available
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU_ADDR, MPU_WHO_AM_I, 1, buffer, 1, MPU_TIMEOUT);

	if (status == HAL_OK) {
		// HAL_OK == 0, so we perform or with all outputs

		// power up mpu
		buffer[0] = 0;
		status = HAL_I2C_Mem_Write(hi2c, MPU_ADDR, MPU_PWR_MGMT_1, 1, buffer, 1, MPU_TIMEOUT);

		if (status == HAL_OK) {
		// set sampling rate
			buffer[0] = MPU_SMPLRT_DIV;
			status = HAL_I2C_Mem_Write(hi2c, MPU_ADDR, MPU_SMPLRT_DIV, 1, buffer, 1, MPU_TIMEOUT);

			if (status == HAL_OK) {
				// configure accelerometer
				buffer[0] = 0;
				status = HAL_I2C_Mem_Write(hi2c, MPU_ADDR, MPU_ACCEL_CONFIG, 1, buffer, 1, MPU_TIMEOUT);

				if (status == HAL_OK) {
					// configure gyroscope
					status = HAL_I2C_Mem_Write(hi2c, MPU_ADDR, MPU_GYRO_CONFIG, 1, buffer, 1, MPU_TIMEOUT);
				}
			}
		}
	}

	return status;
}




HAL_StatusTypeDef MPU_ReadData(I2C_HandleTypeDef * hi2c, MPU_data_t * d)
{
	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
	HAL_StatusTypeDef status = HAL_ERROR;

	if (d != NULL) {
		unsigned char buffer[6];
		status = HAL_I2C_Mem_Read(hi2c, MPU_ADDR, MPU_ACCEL_OUT, 1, buffer, 6, MPU_TIMEOUT);

		if (status == HAL_OK) {
			int16_t accel_X_RAW = (int16_t)(buffer[0] << 8 | buffer [1]);
			int16_t accel_Y_RAW = (int16_t)(buffer[2] << 8 | buffer [3]);
			int16_t accel_Z_RAW = (int16_t)(buffer[4] << 8 | buffer [5]);

			/*** convert the RAW values into acceleration in 'g'
				 we have to divide according to the Full scale value set in FS_SEL
				 I have configured FS_SEL = 0. So I am dividing by 16384.0
				 for more details check ACCEL_CONFIG Register              ****/

			d->accel.x = accel_X_RAW/16384.0;  // get the float g
			d->accel.y = accel_Y_RAW/16384.0;
			d->accel.z = accel_Z_RAW/16384.0;

			status = HAL_I2C_Mem_Read(hi2c, MPU_ADDR, MPU_GYRO_OUT, 1, buffer, 6, MPU_TIMEOUT);

			if (status == HAL_OK) {

				int16_t Gyro_X_RAW = (int16_t)(buffer[0] << 8 | buffer [1]);
				int16_t Gyro_Y_RAW = (int16_t)(buffer[2] << 8 | buffer [3]);
				int16_t Gyro_Z_RAW = (int16_t)(buffer[4] << 8 | buffer [5]);

				/*** convert the RAW values into dps (°/s)
					 we have to divide according to the Full scale value set in FS_SEL
					 I have configured FS_SEL = 0. So I am dividing by 131.0
					 for more details check GYRO_CONFIG Register              ****/

				d->gyro.x = Gyro_X_RAW/131.0;
				d->gyro.y = Gyro_Y_RAW/131.0;
				d->gyro.z = Gyro_Z_RAW/131.0;
			}
		}
	}

	return status;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
