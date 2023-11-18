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
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"

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

/* USER CODE BEGIN PV */
uint8_t Data;
uint8_t Check;
//Let Accel x,y,z
int16_t Accel_X_RAW=0;
int16_t Accel_Y_RAW=0;
int16_t Accel_Z_RAW=0;
float Ax,Ay,Az;
//Let Gyro x,y,z
int16_t Gyro_X_RAW=0;
int16_t Gyro_Y_RAW=0;
int16_t Gyro_Z_RAW=0;
float Gx,Gy,Gz;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Set Data map from mpu9250 data register
#define MPU9250_ADDR 0xD0

#define WHO_AM_I_REG 0x75
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define GYRO_XOUT_H_REG 0x43
#define TEMP_OUT_H_REG 0x41
#define PWR_MGMT_1_REG 0x6B

//Step to know mpu9250
void MPU9250_Init(void)
{
	//Check ID device who am i
	Data=0x00;
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x75, 1, &Check, 1, 1000);
	HAL_Delay(50);
	//Write PWR
	Data=0;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
	HAL_Delay(50);
	//Write SMPLRT
	Data=0x07;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);
	HAL_Delay(50);
	//Write ACCEL
	Data=0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);
	HAL_Delay(50);
	//Write GYRO
	Data=0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	HAL_Delay(50);
}
//Step to read ACCEL MPU9250
void MPU9250_Read_Accel(void)
{
	//Call ACCEL address
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	//ACCEL data
	Accel_X_RAW=(int16_t)(Rec_Data[0]<<8)|Rec_Data[1];
	Accel_Y_RAW=(int16_t)(Rec_Data[2]<<8)|Rec_Data[3];
	Accel_Z_RAW=(int16_t)(Rec_Data[4]<<8)|Rec_Data[5];
	//ACCEL axis
	Ax=Accel_X_RAW/16384.0;
	Ay=Accel_Y_RAW/16384.0;
	Az=Accel_Z_RAW/16384.0;
}
//Step to read GYRO MPU9250
void MPU9250_Read_Gyro()
{
	//Call GYRO address
	uint8_t Rec_Gyro[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, GYRO_XOUT_H_REG, 1, Rec_Gyro, 6, 1000);
	//GYRO data
	Gyro_X_RAW=(int16_t)(Rec_Gyro[0]<<8)|Rec_Gyro[1];
	Gyro_Y_RAW=(int16_t)(Rec_Gyro[2]<<8)|Rec_Gyro[3];
	Gyro_Z_RAW=(int16_t)(Rec_Gyro[4]<<8)|Rec_Gyro[5];
	//GYRO axis
	Gx=Gyro_X_RAW/131.0;
	Gy=Gyro_Y_RAW/131.0;
	Gz=Gyro_Z_RAW/131.0;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  MPU9250_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //Call mpu9250 read accel & Gyro
	  MPU9250_Read_Accel();
	  MPU9250_Read_Gyro();
	  HAL_Delay(10);
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
