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
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include "lcd.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RAD_TO_DEG 57.295779513082320876798154814105
#define PI 3.141592654
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */



#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


//Kalman Struct
typedef struct
{
	double Q_angle;
	double Q_bias;
	double R_measure;
	double angle;
	double bias;
	double P[2][2];
} Kalman_t ;



int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

int16_t Accel_X_Error = 0;
int16_t Accel_Y_Error = 0;
int16_t Accel_Z_Error = 0;

int16_t Gyro_X_Error = 0;
int16_t Gyro_Y_Error = 0;
int16_t Gyro_Z_Error = 0;

float Ax, Ay, Az, Gx, Gy, Gz;

uint32_t timer;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

Kalman_t KalmanX = {
		.Q_angle = 0.001f,
		.Q_bias = 0.003f,
		.R_measure = 0.03f
};

Kalman_t KalmanY = {
		.Q_angle = 0.001f,
		.Q_bias = 0.003f,
		.R_measure = 0.03f
};


double KalmanAngleX;
double KalmanAngleY;

void MPU6050_Init(void)
{
	uint8_t check, data;
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

	if (check == 104) // If the device is present
	{
		//Waking the sensor up
		data = 0;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);


		//Setting Data Rate to 1KHz by writing SMPRT_DIV register
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);


		//Setting accelerometer configuration
		//XA_ST=0, YA_ST=0,ST=0,FS_SEL = 0 -> +-2g
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);


		//Setting Gyroscope configuration
		//XG_ST=0,YG_ST=0, ZG_ST=0,FS_SEL=0 -> +=250 deg/sec
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
	}
}


void MPU6050_Read_Accel(void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	// Converting RAW values into acceleration in g
	// We have to divide according to the Full scale value set in FS_SEL

	Ax = Accel_X_RAW/16484.0;
	Ay = Accel_Y_RAW/16484.0;
	Az = Accel_Z_RAW/16484.0;
}

void MPU6050_Read_Gyro(void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	//Converting RAW  values to degree per second
	// We have to divide according to the full scale value set in FS_SEL

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;
}

void MPU6050_ReadAccelError(void)
{
	uint8_t Rec_Data[6];

		// Read 6 BYTES of data starting from ACCEL_XOUT_H register
		HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_REG, 1, Rec_Data, 6, 1000);

		Accel_X_Error = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
		Accel_Y_Error = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
		Accel_Z_Error = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

		for (int i = 0; i < 200; i++)
		{
			HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_REG, 1, Rec_Data, 6, 1000);

			Accel_X_Error += (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
			Accel_Y_Error += (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
			Accel_Z_Error += (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
			HAL_Delay(100);
		}

		Accel_X_Error = Accel_X_Error/200;
		Accel_Y_Error = Accel_Y_Error/200;
		Accel_Z_Error = Accel_Z_Error/200;

		// Converting RAW values into acceleration in g
		// We have to divide according to the Full scale value set in FS_SEL

		Accel_X_Error = Accel_X_Error/16484.0;
		Accel_Y_Error = Accel_Y_Error/16484.0;
		Accel_Z_Error = Accel_Z_Error/16484.0;
}

void MPU6050_ReadGryoError(void)
{
	uint8_t Rec_Data[6];

		// Read 6 BYTES of data starting from GYRO_XOUT_H register
		HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

		Gyro_X_Error = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
		Gyro_Y_Error = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
		Gyro_Z_Error = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

		for (int i = 0; i < 200; i++)
		{
			HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

			Gyro_X_Error += (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
			Gyro_Y_Error += (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
			Gyro_Z_Error += (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

			HAL_Delay(100);
		}

		Gyro_X_Error = Gyro_X_Error/200;
		Gyro_Y_Error = Gyro_Y_Error/200;
		Gyro_Z_Error = Gyro_Z_Error/200;

		//Converting RAW  values to degree per second
		// We have to divide according to the full scale value set in FS_SEL

		Gyro_X_Error = Gyro_X_Error/131.0;
		Gyro_Y_Error = Gyro_Y_Error/131.0;
		Gyro_Z_Error = Gyro_Z_Error/131.0;

}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};



void MPU6050_Read_All()
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	// Converting RAW values into acceleration in g
	// We have to divide according to the Full scale value set in FS_SEL

	Ax = Accel_X_RAW/16484.0;
	Ay = Accel_Y_RAW/16484.0;
	Az = Accel_Z_RAW/16484.0;

	uint8_t Rec_Data2[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data2, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data2[0] << 8 | Rec_Data2[1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data2[2] << 8 | Rec_Data2[3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data2[4] << 8 | Rec_Data2[5]);

	//Converting RAW  values to degree per second
	// We have to divide according to the full scale value set in FS_SEL

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;


	// Kalman Angle Solve
	double dt = (double)(HAL_GetTick() - timer) / 1000;
	timer = HAL_GetTick();

	double roll;
	double roll_sqrt = sqrt(Accel_X_RAW * Accel_X_RAW + Accel_Z_RAW * Accel_Z_RAW);

	if (roll_sqrt != 0)
	{
		roll = atan(Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
	}
	else
	{
		roll = 0.0;
	}

	double pitch = atan2(-Accel_X_RAW,Accel_Z_RAW) * RAD_TO_DEG;
	if ((pitch < -90 && KalmanAngleY > 90) || (pitch > 90 && KalmanAngleY < -90))
	{
		KalmanY.angle = pitch;
		KalmanAngleY = pitch;
	}
	else
	{
		KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, Gy, dt);

	}
	if (fabs(KalmanAngleY) > 90)
	{
		Gx = -Gx;
	}

	KalmanAngleX = Kalman_getAngle(&KalmanX, roll, Gx, dt);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */
	char buf[4];
	  /* USER CODE END 1 */

	  /* MCU Configuration--------------------------------------------------------*/

	  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	//  __HAL_RCC_I2C2_CLK_ENABLE();
	  HAL_Init();

	  /* USER CODE BEGIN Init */

	  /* USER CODE END Init */

	  /* Configure the system clock */
	  SystemClock_Config();

	  /* USER CODE BEGIN SysInit */

	  /* USER CODE END SysInit */

	  /* Initialize all configured peripherals */
	  MX_GPIO_Init();
	  MX_FSMC_Init();
	  MX_I2C2_Init();
	  LCD_INIT();
	  /* USER CODE BEGIN 2 */
	  LCD_INIT();
	  MPU6050_Init();

	  LCD_DrawString(20,20, "Initialized");

	  HAL_Delay(1000);

	  LCD_DrawString(50, 20, "MPU6050");




	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
	    /* USER CODE END WHILE */



	    /* USER CODE BEGIN 3 */
		  MPU6050_Read_Accel();
		  MPU6050_Read_Gyro();
		  MPU6050_Read_All();

		 //Printing on LCD
		  sprintf(buf, "%.2f",Gx);
		  LCD_DrawString(100,100, buf);

		  sprintf(buf, "%.2f",Gy);
		  LCD_DrawString(100,200, buf);

		  sprintf(buf, "%.2f",Gz);
		  LCD_DrawString(100,300, buf);

		  sprintf(buf, "%.2f",KalmanAngleX);
		  LCD_DrawString(90, 50, buf);

		  sprintf(buf, "%.2f",KalmanAngleY);
		  LCD_DrawString(90,90, buf);


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

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
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
