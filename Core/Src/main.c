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
//#include "stm32l1xx_hal_iwdg.h"
#include "stm32l1xx_hal.h"
#include "stm32l1xx.h"
#include "stm32l1xx_it.h"
#include "stm32l1xx_hal_tim.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "console.h"
#include "usart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int32_t file, uint8_t *ptr, int32_t len) {
  if(HAL_UART_Transmit(&huart1, ptr, len, 10) == HAL_OK) return len;
  else return 0;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
IWDG_HandleTypeDef hiwdg;

uint8_t rx_data;
uint8_t host_rx_data;
uint8_t rfid_rx_data;

int SystemTime;
int prevTemperatureLevel;
PCD_HandleTypeDef hpcd_USB_FS;
uint8_t LED_Mode=0;
uint8_t LED_Data=0;
uint32_t g_1ms_cnt =0;
uint8_t g_1ms_flag =0;
uint32_t g_2ms_cnt =0;
uint8_t g_2ms_flag =0;
uint32_t g_10ms_cnt =0;
uint8_t g_10ms_flag =0;
uint32_t g_50ms_cnt =0;
uint8_t g_50ms_flag =0;
uint32_t g_100ms_cnt =0;
uint8_t g_100ms_flag =0;
uint32_t g_500ms_cnt =0;
uint8_t g_500ms_flag =0;
uint32_t g_1sec_cnt =0;
uint8_t g_1sec_flag =0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef HAL_I2C_Temperature_PCT2075_Read(uint16_t MemAddress, uint8_t * pData, uint16_t Size, uint32_t Timeout) 
{
    return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)I2C_SLAVE_ADDRESS__THERMISTOR_PCT2075, MemAddress, (uint16_t)I2C_MEMADD_SIZE_8BIT, pData, Size, Timeout);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */
  //lsi : 37Khz
  // 1/37k * 4 * 4096 --> 약 400ms 
  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if(htim->Instance == htim2.Instance) {
    //printf("\r\ntesttest\r\n");
    g_1ms_flag =1;
		g_2ms_cnt++;
		if(g_2ms_cnt >= 2)  {
			g_2ms_flag = 1;
			g_10ms_cnt++;
			g_2ms_cnt =0;
		}
		if(g_10ms_cnt >= 5)	{
			g_50ms_cnt++;
			g_10ms_flag =1;
			g_10ms_cnt =0;
		}
		if(g_50ms_cnt >= 5) {
			g_100ms_cnt++;
			g_50ms_flag =1;
			g_50ms_cnt =0;		
		}
		if(g_100ms_cnt >= 2) {
			g_500ms_cnt++;
			g_100ms_flag =1;
			g_100ms_cnt =0;
		}
    if(g_500ms_cnt >= 5) {
      g_1sec_cnt++;
      g_500ms_flag =1;
      g_500ms_cnt =0;
    }
		if(g_1sec_cnt >= 2) {
			g_1sec_flag =1;
			g_1sec_cnt =0;
		}		
  }
}

void StatusLED(uint8_t mode)
{
	LED_Mode++;
	if(mode == 1) {
		LED_Data = 0x80;
		LED_Mode &= 0x07;
	}
	else if (mode == 2) {
		LED_Data = 0xA0;
		LED_Mode &= 0x07;
	}
	
	if (LED_Data & (0x80 >> LED_Mode))	HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, GPIO_PIN_SET);
	else								                HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, GPIO_PIN_RESET);
}

//100ms
void HAL_SYSTICK_Callback(void)
{

}


uint8_t prev_sensor =0;
uint8_t curr_sensor =0;

uint8_t compare_sensor(void)
{
  curr_sensor = HAL_GPIO_ReadPin(PIR_SENSOR_Port, PIR_SENSOR);
  if(prev_sensor != curr_sensor) {
    if(curr_sensor) {
      printf("\r\ndetected sensor");
    }
    else {
      printf("\r\nrelease sensor");
    }
    event_comm_detect_sensor(curr_sensor);
    prev_sensor = curr_sensor;
  }
  return 0;
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
  //TIM3 : LIGHT_PWM
  //I2C1 : Temperature sensor
  //USART1 : console debug
  //USART2 :  
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_USB_PCD_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  usart_queue_init();
  printf("\r\nusart1 initial done.");
  printf("\r\nusart2 initial done.");
  printf("\r\nusart3 initial done.");
  printf("\r\nsoftware started.");
  print_SystemClock();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
  MX_IWDG_Init();
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  HAL_UART_Receive_IT(&huart2, &host_rx_data, 1);
  HAL_UART_Receive_IT(&huart4, &rfid_rx_data, 1);
  MX_IWDG_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    read_usart();
    if(g_100ms_flag) {
      StatusLED(2);
      HAL_IWDG_Refresh(&hiwdg);
      g_100ms_flag =0;
    }
    if (g_500ms_flag) {
      g_500ms_flag =0;
      compare_sensor();
    }
    if(g_1sec_flag) {
      EnvacIO.temperature = GetThermistorLevel();
      g_1sec_flag =0;
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 10000;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}


/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void print_SystemClock(void)
{
  printf("\r\nHCLK=%luHz", HAL_RCC_GetHCLKFreq());
  printf("\r\nAPB1=%luHz", HAL_RCC_GetPCLK1Freq());
  printf("\r\nAPB2=%luHz", HAL_RCC_GetPCLK2Freq());
}

void proc_SysLed(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

int SetLightBright(int value)
{
  if(value < 0 || value > 100) {
    EnvacIO.light_bright =0;
    value = 0;
    return -1;
  }
  else {
    EnvacIO.light_bright = value;
    TIM3->CCR1 = value*10;
    return 0;
  }
}

int8_t GetThermistorLevel(void) 
{
    HAL_StatusTypeDef state;
    int8_t compute;
    uint8_t temperature[THERMISTOR_DATA_SIZE];
    uint16_t temp;

    //DEBUG_FUNC("I2C ERROR"); DEBUG("%d\r\n", state);
    state = HAL_I2C_Temperature_PCT2075_Read(THERMISTOR_ADDRESS_DATA, temperature, THERMISTOR_DATA_SIZE, 500);
    if(HAL_OK != state) {
        printf("\r\nI2C ERROR %d", state);
        return prevTemperatureLevel;
    } else {
        temp = (temperature[0] << 3) | (temperature[1] >> 5);
        if((temp & 0x400) == 0x400) {
            temp = (temp ^ 0x07FF) + 1;
            compute = (uint8_t)((double)temp * 0.125);
            compute = -compute;
        } else {
            compute = (uint8_t)((double)temp * 0.125);
        }
        prevTemperatureLevel = compute;
        //printf("\r\nTEMPERATURE LEVEL(%d C)", compute);
    
        return compute;
    }
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
    HAL_NVIC_SystemReset();
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
     printf("\r\n****Wrong parameters value: file %s on line %d****\r\n", file, line);
     HAL_NVIC_SystemReset();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
