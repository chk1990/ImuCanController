/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.cpp
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
#include <stdlib.h>
#include "Lsm9ds1.h"

// - In timer interrupt callback acquire data
// - After finishing acquisition transfer data via CAN

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_VAL_ACC_X 20
#define N_VAL_ACC_Y 20
#define N_VAL_ACC_Z 20
#define N_VAL_GYR_X 20
#define N_VAL_GYR_Y 20
#define N_VAL_GYR_Z 20

#define N_VAL_BUF_IN_ACC_X 20
#define N_VAL_BUF_IN_ACC_Y 20
#define N_VAL_BUF_IN_ACC_Z 20
#define N_VAL_BUF_IN_GYR_X 20
#define N_VAL_BUF_IN_GYR_Y 20
#define N_VAL_BUF_IN_GYR_Z 20

#define N_VAL_BUF_OUT_ACC_X 20
#define N_VAL_BUF_OUT_ACC_Y 20
#define N_VAL_BUF_OUT_ACC_Z 20
#define N_VAL_BUF_OUT_GYR_X 20
#define N_VAL_BUF_OUT_GYR_Y 20
#define N_VAL_BUF_OUT_GYR_Z 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
Lsm9ds1 Imu;
int16_t *accX; ///< Buffer for acceleration in x-direction
int16_t *accY; ///< Buffer for acceleration in y-direction
int16_t *accZ; ///< Buffer for acceleration in z-direction
int16_t *gyrX; ///< Buffer for angular velocity around x-axis
int16_t *gyrY; ///< Buffer for angular velocity around y-axis
int16_t *gyrZ; ///< Buffer for angular velocity around z-axis

float sensAcc = 0.0; ///< Sensitivity of the accelerometer
float sensGyr = 0.0; ///< Sensitivity of the gyroscope

float *kernelAccXIn;  ///< Elements of the input kernel of the accelerometer in x-direction
float *kernelAccYIn;  ///< Elements of the input kernel of the accelerometer in y-direction
float *kernelAccZIn;  ///< Elements of the input kernel of the accelerometer in z-direction
float *kernelGyrXIn;  ///< Elements of the input kernel of the gyroscope around the x-axis
float *kernelGyrYIn;  ///< Elements of the input kernel of the gyroscope around the y-axis
float *kernelGyrZIn;  ///< Elements of the input kernel of the gyroscope around the z-axis
float *kernelAccXOut; ///< Elements of the output kernel of the accelerometer in x-direction
float *kernelAccYOut; ///< Elements of the output kernel of the accelerometer in y-direction
float *kernelAccZOut; ///< Elements of the output kernel of the accelerometer in z-direction
float *kernelGyrXOut; ///< Elements of the output kernel of the gyroscope around the x-axis
float *kernelGyrYOut; ///< Elements of the output kernel of the gyroscope around the y-axis
float *kernelGyrZOut; ///< Elements of the output kernel of the gyroscope around the z-axis

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void initVars(void);
void initImu(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//HAL_GPIO_TogglePin(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin);

	/*
	if(TIM2 == htim->Instance) {
		if(HAL_TIM_ACTIVE_CHANNEL_2 == htim->Channel) {
			//
		}
	}
	 */
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
	MX_CAN_Init();
	MX_TIM2_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	Imu.init(&hi2c1, 0xD4, 0x38);
	HAL_TIM_Base_Start_IT(&htim2);

	uint8_t value = Imu.whoAmI();
	if(104 == value) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	}

	initVars();
	initImu();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//HAL_GPIO_TogglePin(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin);
		//HAL_Delay(500);
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 16;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

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
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 9;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 63999;
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
	if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

	/*Configure GPIO pins : SPI1_CS_IMU_Pin PA10 */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * @brief Allocate memory for arrays and populate with values
 */
void initVars(void)
{
	accX = (int16_t*) calloc(N_VAL_ACC_X, sizeof(int16_t));
	accY = (int16_t*) calloc(N_VAL_ACC_Y, sizeof(int16_t));
	accZ = (int16_t*) calloc(N_VAL_ACC_Z, sizeof(int16_t));

	gyrX = (int16_t*) calloc(N_VAL_GYR_X, sizeof(int16_t));
	gyrY = (int16_t*) calloc(N_VAL_GYR_Y, sizeof(int16_t));
	gyrZ = (int16_t*) calloc(N_VAL_GYR_Z, sizeof(int16_t));

	kernelAccXIn = (float*) calloc(N_VAL_BUF_IN_ACC_X, sizeof(float));
	kernelAccYIn = (float*) calloc(N_VAL_BUF_IN_ACC_Y, sizeof(float));
	kernelAccZIn = (float*) calloc(N_VAL_BUF_IN_ACC_Z, sizeof(float));
	kernelGyrXIn = (float*) calloc(N_VAL_BUF_IN_GYR_X, sizeof(float));
	kernelGyrYIn = (float*) calloc(N_VAL_BUF_IN_GYR_Y, sizeof(float));
	kernelGyrZIn = (float*) calloc(N_VAL_BUF_IN_GYR_Z, sizeof(float));

	kernelAccXOut = (float*) calloc(N_VAL_BUF_OUT_ACC_X, sizeof(float));
	kernelAccYOut = (float*) calloc(N_VAL_BUF_OUT_ACC_Y, sizeof(float));
	kernelAccZOut = (float*) calloc(N_VAL_BUF_OUT_ACC_Z, sizeof(float));
	kernelGyrXOut = (float*) calloc(N_VAL_BUF_OUT_GYR_X, sizeof(float));
	kernelGyrYOut = (float*) calloc(N_VAL_BUF_OUT_GYR_Y, sizeof(float));
	kernelGyrZOut = (float*) calloc(N_VAL_BUF_OUT_GYR_Z, sizeof(float));
}

/**
 * @brief
 */
void initImu(void)
{
	//
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
