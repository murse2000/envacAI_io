/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void print_SystemClock(void);
int8_t GetThermistorLevel(void) ;
int SetLightBright(int value);

#define HAL_IWDG_MODULE_ENABLED

#define SYS_LED_GPIO_Port GPIOB 
#define SYS_LED_Pin       GPIO_PIN_0

#define PIR_SENSOR        GPIO_PIN_13
#define PIR_SENSOR_Port   GPIOB
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

extern int prevTemperatureLevel;

#define THERMISTOR_DATA_SIZE    2
#define THERMISTOR_ADDRESS_DATA 0x00
#define THERMISTOR_ADDRESS_REG  0x01
#define THERMISTOR_ADDRESS_LOW  0x02
#define THERMISTOR_ADDRESS_HIGH 0x03

#define I2C_SLAVE_ADDRESS__THERMISTOR_TMP102   0x90
#define I2C_SLAVE_ADDRESS__THERMISTOR_PCT2075  0x92

extern uint8_t rx_data;
extern uint8_t host_rx_data;
extern uint8_t rfid_rx_data;

int SetLightBright(int value);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
