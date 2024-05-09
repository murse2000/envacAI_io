/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : console.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 bnc-tech.
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
#ifndef __CONSOLE_H
#define __CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#define U1_BUFFER_SIZE	100

void Uart1Queue_Initialize(void);
void Uart1_EnQueue(uint8_t data);
uint8_t Uart1_DeQueue(void);
void usart_console(uint8_t data);
void u1_increase_point_value(uint32_t * data_p);
void uart_putc(uint8_t dat, UART_HandleTypeDef *huart);
void Console_ReadUart(void);
void hex_dump(const void *src, size_t length, size_t line_size, char *prefix);
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
