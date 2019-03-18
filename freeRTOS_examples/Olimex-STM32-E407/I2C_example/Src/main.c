
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "stdlib.h"

/* USER CODE BEGIN Includes */
 #define HIH6130_ADDRESS 0x27
 #define HIH6130_FREQ         100000
 #define HIH6130_READREG 0x00

/* USER CODE END Includes */



/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  //This thread check the temperature every second

  for(;;)
  {
	humidity_temp();
	osDelay(1000);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

static uint8_t hih6130_getreg8(I2C_HandleTypeDef * hi2c, uint8_t regaddr)
{
  uint8_t regval;
  int ret;

  /*Write the register address*/
  ret=HAL_I2C_Master_Transmit (&hi2c1, HIH6130_ADDRESS<<1,&regaddr, 1, 100);
  if(ret>0){
    //error
    printf("GetReg8 error transmitting: %d \r\n",ret);
    return 0;
  }

  //Read the register value

  ret=HAL_I2C_Master_Receive (&hi2c1,HIH6130_ADDRESS<<1,regval,1,100);
  if(ret>0){
    //error
	printf("GetReg8 error receiving: %d \r\n",ret);
	return 0;
  }
  return regval;
}

static uint16_t hih6130_getreg16(I2C_HandleTypeDef * hi2c, uint8_t regaddr)
{
  uint16_t msb, lsb;
  uint16_t regval = 0;
  int ret;

  /*Write the register address*/
  ret=HAL_I2C_Master_Transmit (&hi2c1, HIH6130_ADDRESS<<1,&regaddr, 1, 100);
  if(ret>0){
    //error
	printf("GetReg16 error transmitting: %d \r\n",ret);
	return 0;
  }

  //Read the register value

  ret=HAL_I2C_Master_Receive (&hi2c1,HIH6130_ADDRESS<<1,regval,2,100);
  if(ret>0){
	//error
	printf("GetReg16 error receiving: %d \r\n",ret);
	return 0;
  }

  msb = (regval & 0xFF);
  lsb = (regval & 0xFF00) >> 8;

  regval = (msb << 8) | lsb;

  return regval;
}

static uint32_t hih6130_getreg32( uint8_t regaddr)//modify the arguments
{
  uint16_t byte1, byte2, byte3, byte4;//byte1 is the msb and byte4 is the lsb
  uint16_t msb, lsb;
  uint32_t regval = 0;
  int ret;

  /*Write the register address*/
  ret=HAL_I2C_Master_Transmit (&hi2c1, HIH6130_ADDRESS<<1,&regaddr, 1, 100);
  if(ret>0){
    //error
	  printf("Error getreg32 transmit: %i \r\n",ret);
  }

  //Read the register value

  ret=HAL_I2C_Master_Receive (&hi2c1,HIH6130_ADDRESS<<1,&regval,4,100);
  if(ret>0){
   printf("Error getreg32 receive: %i \r\n",ret);
  }

  byte1=(regval & 0xFF);
  byte2=(regval & 0xFF00) >>8;
  byte3=(regval & 0xFFFF00) >> 16;
  byte4=(regval & 0xFFFFFF00) >> 24;

  regval=(byte1 << 24) | (byte2 << 16) | (byte3 << 8) | byte4;

  return regval;
}

static void hih6130_putreg8(I2C_HandleTypeDef * hi2c, uint8_t regaddr,
                           uint8_t regval)
{
  uint8_t data[2];
  int ret;

  data[0] = regaddr;
  data[1] = regval;

  /* Write the register address and value */
  HAL_I2C_Master_Transmit (&hi2c1, HIH6130_ADDRESS<<1,&regaddr, 1, 100);
  ret=HAL_I2C_Master_Transmit (&hi2c1, HIH6130_ADDRESS<<1,(uint8_t *) &data, 2, 100);

  return ;
}

void humidity_temp(){

	uint32_t regval0 ;
	uint32_t data = 0;
	uint8_t sensor_status=0;
	uint16_t humidity_data,humidity = 0;
	int16_t temperature_data, temperature = 0;

	data = hih6130_getreg32( HIH6130_READREG);

	//This is the data that we recieve from the device
	//Are four bytes
	//First we check the state of the sensor (First two bits)
	sensor_status=(data & 0xFFFFFF00) >> 30;

	if(sensor_status == 0 || sensor_status == 1){
	//sensor status = 0, sensor is working correctly
	//sensor status = 1, sensor is working correctly but you should reduce the measure frequency
	//because the data that is showing is from the buffer
	//Getting the humidity data
	humidity_data = (data & 0xFFFFFF00) >> 16;

	humidity_data = (humidity_data) << 2;
	humidity_data = (humidity_data) >> 2;

	humidity=humidity_data*100/(16384 -1);
	printf("humidity value: %i \r\n",humidity);
	//Getting the temperature data
	temperature_data = (data & 0xFFFF) >> 2;

	temperature = temperature_data*165/(16384-1) - 40;

	printf("temperature value: %i \r\n",temperature);
}
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
