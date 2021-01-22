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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define address 0b01111000

#define R 0
#define G 1
#define B 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void WRITE_I2C(uint8_t reg, uint8_t data);
void UPDATE_REGISTER(void);
void Init_FL3236A(void);
void LED(uint8_t LED, uint8_t color, uint8_t value);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void LED_MODE(uint8_t mode);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const uint8_t PWM_REG[12][3] = {{0x1,0x2,0x3},
						  {0x4,0x5,0x6},
						  {0x7,0x8,0x9},
						  {0xA,0xB,0xC},
						  {0xD,0xE,0xF},
						  {0x10,0x11,0x12},
						  {0x22,0x23,0x24},
						  {0x1F,0x20,0x21},
						  {0x1C,0x1D,0x1E},
						  {0x19,0x1A,0x1B},
						  {0x16,0x17,0x18},
						  {0x13,0x14,0x15}};

const uint8_t G_CONTROL_REG = 0x4A;
const uint8_t PWM_UPDATE_REG = 0x25;
const uint8_t SHUTDOWN_REG = 0x00;
const uint8_t RESET_REG = 0x4F;

volatile uint8_t state = 1;
/*const uint8_t SIN[180] = {0,1,2,4,5,7,8,10,11,13,14,15,16,18,19,20,21,22,23,24,25,26,27,27,28,28,28,29,29,29,29,29,29,28,28,28,27,27,26,25,24,23,22,21,20,19,18,16,15,14,13,11,10,8,7,5,4,2,1,0,
						  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
						  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
*/
const unsigned char SIN2[64] = { 0,0,0,1,2,4,5,7,8,10,11,13,14,15,16,18,19,20,21,22,23,24,25,26,27,27,28,28,28,29,29,29,29,29,29,28,28,28,27,27,26,25,24,23,22,21,20,19,18,16,15,14,13,11,10,8,7,5,4,2,1,0,0,0};
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

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(I2C_EN_GPIO_Port,I2C_EN_Pin, GPIO_PIN_SET);

  if(HAL_I2C_IsDeviceReady(&hi2c1, address, 2, 10) == HAL_OK)
  {
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
  }
  else
  {
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
  }

  uint8_t counter = 0;
  uint8_t oldstate = 0;
 /* WRITE_I2C(PWM_REG[7][0],64); //LED8 BLAU, PWM=64
  WRITE_I2C(CONTROL_REG[7][0], 0b111); //LED8 BLAU, IMAX/4 & LED ON

  UPDATE_REGISTER();*/

  Init_FL3236A();

/*
    uint8_t reset[2] = {0x4F, 0x00};
	HAL_I2C_Master_Transmit(&hi2c1, address, reset, 2, 10);

  	uint8_t newData[2] = {0x01, 64};
  	HAL_I2C_Master_Transmit(&hi2c1, address, newData, 2, 10);

  	uint8_t newData2[2] = {0x26, 0x07};
  	HAL_I2C_Master_Transmit(&hi2c1, address, newData2, 2, 10);

  	uint8_t newData3[3] = {0x25, 0x00};
  	HAL_I2C_Master_Transmit(&hi2c1, address, newData3, 2, 10);
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 if(state != oldstate)
	 {
		 for(counter = 1; counter != 13; counter++)
		 {
			 LED(counter,R,0);
			 LED(counter,G,0);
			 LED(counter,B,0);
		 }
		 oldstate = state;
	 }
	 LED_MODE(state);
	 HAL_Delay(100);
	  /*if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port,BUTTON_Pin))
	  {
		  HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
	  }
	  else
	  {
		 // HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
	  }*/
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI4_15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
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
  hi2c1.Init.Timing = 0x00707CBB;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|I2C_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C_EN_Pin */
  GPIO_InitStruct.Pin = I2C_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void LED_MODE(uint8_t mode)
{
	static uint8_t i[12] = {15,40,50,10,25,5,30,0,35,55,20,45};

	uint8_t counter = 0;
	//static uint8_t LEDvalues = 0;

	switch(mode)
	{
		case 1:

			LED(2,R,SIN2[i[2-1]]);
			LED(4,R,SIN2[i[4-1]]);
			LED(7,R,SIN2[i[7-1]]);
			LED(11,R,SIN2[i[11-1]]);

			LED(5,G,SIN2[i[5-1]]);
			LED(1,G,SIN2[i[1-1]]);
			LED(9,G,SIN2[i[9-1]]);
			LED(12,G,SIN2[i[12-1]]);

			LED(8,B,SIN2[i[8-1]]);
			LED(3,B,SIN2[i[3-1]]);
			LED(6,B,SIN2[i[6-1]]);
			LED(10,B,SIN2[i[10-1]]);
			break;
		case 2:
			for(counter = 1; counter != 13; counter++)
			{
				LED(counter,R,20);
				LED(counter,G,16);
			}
			break;
		case 3:
			for(counter = 1; counter != 13; counter++)
			{
				LED(counter,R,30);
			}
			break;
		case 4:
			for(counter = 1; counter != 13; counter++)
			{
				LED(counter,G,30);
			}
			break;
		case 5:
			for(counter = 1; counter != 13; counter++)
			{
				LED(counter,B,30);
			}
			break;
			//LED(1,R,);
	}

	for(counter = 0; counter != 12; counter++)
	{
		if(i[counter]>=63)
		{
			i[counter] = 0;
		}
		else
		{
			i[counter]++;
		}
	}

	UPDATE_REGISTER();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BUTTON_Pin)
	{
		volatile int i = 0;
		for(i=0; i != 50000; i++);
		//HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		if(state >= 5)
		{
			state = 1;
		}
		else
		{
			state++;
		}
	}
}

void UPDATE_REGISTER(void)
{
	WRITE_I2C(PWM_UPDATE_REG,0x00);
}

void WRITE_I2C(uint8_t reg, uint8_t data)
{
	uint8_t newData[2] = {reg, data};

	HAL_I2C_Master_Transmit(&hi2c1, address, newData, 2, 10);
}

void LED(uint8_t LED, uint8_t color, uint8_t value)
{
	if(LED == 0) LED++;
	if(color > 2) color = 2;
	if(value > 30) value = 30;

	WRITE_I2C(PWM_REG[LED-1][color], value);
}

void Init_FL3236A(void)
{
	uint8_t i = 0;
	uint8_t data[2];

	data[0] = 0x4F;
	data[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, address, data, 2, 10);
	//IS_IIC_WriteByte(Addr_VCC,0x25,0x00);//update PWM & congtrol registers

	HAL_Delay(500);

	data[0] = 0x4A;
	data[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, address, data, 2, 10);
	//IS_IIC_WriteByte(Addr_VCC,0x25,0x00);//update PWM & congtrol registers

	data[0] = 0x00;
	data[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, address, data, 2, 10);
	//IS_IIC_WriteByte(Addr_VCC,0x00,0x01);//normal operation

	data[1] = 0x7;
	for(i=0x26;i<=0x49;i++)
	{
		data[0] = i;
		HAL_I2C_Master_Transmit(&hi2c1, address, data, 2, 10);
		//IS_IIC_WriteByte(Addr_VCC,i,0xff);//turn on all LED
	}

	data[1] = 0x00;
	for(i=0x01;i<=0x24;i++)
	{
		data[0] = i;
		HAL_I2C_Master_Transmit(&hi2c1, address, data, 2, 10);
		//IS_IIC_WriteByte(Addr_VCC,i,0x00);//write all PWM set 0x00
	}

	data[0] = 0x25;
	data[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, address, data, 2, 10);
	//IS_IIC_WriteByte(Addr_VCC,0x25,0x00);//update PWM & congtrol registers


	/*data[0] = 0x4B;
	data[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, address, data, 2, 10);
	//IS_IIC_WriteByte(Addr_VCC,0x4B,0x01);//frequency setting 22KHz*/


	HAL_Delay(100);

	/*data[1] = 30;
	for(i=0x01;i<=0x24;i++)
	{
		data[1] = 90;
		data[0] = 0x19;
		HAL_I2C_Master_Transmit(&hi2c1, address, data, 2, 10);
		//IS_IIC_WriteByte(Addr_VCC,i,0x00);//write all PWM set 30

		data[0] = 0x25;
		data[1] = 0x00;
		HAL_I2C_Master_Transmit(&hi2c1, address, data, 2, 10);
		//IS_IIC_WriteByte(Addr_VCC,0x25,0x00);//update PWM & congtrol registers

		HAL_Delay(100);
	}*/


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
