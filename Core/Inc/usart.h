/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usart.h
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
#ifndef __USART_H
#define __USART_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "project.h"
#define STX     0x02
#define ETX     0x03

#define HOST_CMD_REQ_LIGHT_BRIGHT 0x01
#define HOST_CMD_REP_LIGHT_BRIGHT 0x81
#define HOST_CMD_EVENT_PIR_IN     0x21
#define HOST_CMD_REQ_GATE_OPEN    0x02
#define HOST_CMD_REP_GATE_OPEN    0x82
#define HOST_CMD_EVENT_RFID_IN    0x22
#define HOST_CMD_REQ_TEMP         0x03
#define HOST_CMD_REP_TEMP         0x83


//protocol structure
//  STX     LEN     CMD     MSG     CRC     ETX
// 0x02   STX-ETX                           0x03
#define POS_STX     0
#define POS_LEN     1
#define POS_CMD     2
#define POS_MSG     3


#define COMM_IDLE       0
#define COMM_LENGTH     1
#define COMM_CMD        2
#define COMM_MSG        3

#define RFID_COMM_LEN   12

#define PIR_DETECT    0x01
#define PIR_RELEASE   0x00

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern EnvacIOStruct EnvacIO;

void MX_UART4_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void uart_tx(UART_HandleTypeDef *huart, uint8_t len, uint8_t *tx_buf);
uint8_t makeBasePacket(uint8_t cmd, uint8_t dlen, uint8_t *msg_data);
uint8_t Calc_Crc(uint8_t dlen, uint8_t *data);
void uart_tx(UART_HandleTypeDef *huart, uint8_t len, uint8_t *tx_buf);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t rep_comm_bright(void);
uint8_t Uart2_Is_Empty(void);
void Uart2_EnQueue(uint8_t data);
uint8_t Uart2_DeQueue(void);
void u2_increase_point_value(uint32_t * data_p);
void Uart2Queue_Initialize(void);
uint8_t proc_comm(uint8_t cmd);
void read_usart(void);
uint8_t event_comm_detect_sensor(uint8_t sen_in);
uint8_t rep_comm_gate_open(void);

uint8_t Uart4_Is_Empty(void);
void Uart4_EnQueue(uint8_t data);
uint8_t Uart4_DeQueue(void);
void u4_increase_point_value(uint32_t * data_p);
void Uart4Queue_Initialize(void);
void usart_queue_init(void);
uint8_t rep_comm_temerature(void);
uint8_t event_comm_rfid_in(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

