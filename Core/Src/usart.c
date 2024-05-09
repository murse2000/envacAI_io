
#include "usart.h"
/* USER CODE BEGIN Includes */
#include "stm32l1xx_hal.h"
#include "stm32l1xx.h"
#include "stm32l1xx_it.h"
#include "stm32l1xx_hal_uart.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "console.h"
#include "main.h"
#include "project.h"

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

#define U4_BUFFER_SIZE 1024
#define U2_BUFFER_SIZE 1024

uint32_t  u2_rx_point_head = 0;
uint32_t  u2_rx_point_tail = 0;
uint8_t u2_rx_buffer[U2_BUFFER_SIZE] = {0, };

uint32_t  u4_rx_point_head = 0;
uint32_t  u4_rx_point_tail = 0;
uint8_t u4_rx_buffer[U4_BUFFER_SIZE] = {0, };

uint8_t rcv_mode=0;
uint8_t rfid_rcv_mode =0;
uint8_t rfid_len=0;
uint8_t rfid_buff[256];
uint8_t p_len =0;
uint8_t msg_len=0;
uint8_t rcv_cmd=0;
uint8_t rcv_buf[256];
EnvacIOStruct EnvacIO;

void usart_queue_init(void)
{
  Uart1Queue_Initialize();
  Uart2Queue_Initialize();
  Uart4Queue_Initialize();
}


uint8_t Uart4_Is_Empty(void)
{
	if(u4_rx_point_head == u4_rx_point_tail) 	return 1; 
	else 										return 0; 
}

void Uart4_EnQueue(uint8_t data)
{
	u4_rx_buffer[u4_rx_point_head] = data;
	u4_increase_point_value(&u4_rx_point_head);
}

uint8_t Uart4_DeQueue(void)
{
	uint8_t retVal = u4_rx_buffer[u4_rx_point_tail];
	u4_increase_point_value(&u4_rx_point_tail);
	
	return retVal;
}

void u4_increase_point_value(uint32_t * data_p)
{
	(* data_p)++;
	if(U4_BUFFER_SIZE == (* data_p)) {
		(* data_p) = 0;
	}
}

void Uart4Queue_Initialize(void)
{
	u4_rx_point_head = u4_rx_point_tail = 0;
}





uint8_t Uart2_Is_Empty(void)
{
	if(u2_rx_point_head == u2_rx_point_tail) 	return 1; 
	else 										return 0; 
}

void Uart2_EnQueue(uint8_t data)
{
	u2_rx_buffer[u2_rx_point_head] = data;
	u2_increase_point_value(&u2_rx_point_head);
}

uint8_t Uart2_DeQueue(void)
{
	uint8_t retVal = u2_rx_buffer[u2_rx_point_tail];
	u2_increase_point_value(&u2_rx_point_tail);
	
	return retVal;
}

void u2_increase_point_value(uint32_t * data_p)
{
	(* data_p)++;
	if(U2_BUFFER_SIZE == (* data_p)) {
		(* data_p) = 0;
	}
}

void Uart2Queue_Initialize(void)
{
	u2_rx_point_head = u2_rx_point_tail = 0;
}





void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1) {
    usart_console(rx_data);
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
  if(huart->Instance == USART2) {
    Uart2_EnQueue(host_rx_data);
    HAL_UART_Receive_IT(&huart2, &host_rx_data, 1);
  }
  if(huart->Instance == UART4) {
    Uart4_EnQueue(rfid_rx_data);
    HAL_UART_Receive_IT(&huart4, &rfid_rx_data, 1);
  }
}


void uart_tx(UART_HandleTypeDef *huart, uint8_t len, uint8_t *tx_buf)
{
	HAL_UART_Transmit(huart, tx_buf, len, 100);
}


uint8_t Calc_Crc(uint8_t dlen, uint8_t *data)
{
    uint8_t SumData =0;
    uint8_t cnt =0;
    
    for(cnt=0;cnt<dlen;cnt++) SumData ^= (uint8_t)*(data+cnt);

    return SumData;
}

uint8_t makeBasePacket(uint8_t cmd, uint8_t dlen, uint8_t *msg_data)
{
  uint8_t cnt =0;
  uint8_t len =0;
    
  EnvacIO.tx_data[len++] =    STX;
  EnvacIO.tx_data[len++] =    dlen+5;  //stx + len + cmd + ecc + etx + dlen
  EnvacIO.tx_data[len++] =    cmd;
  for(cnt=0;cnt<dlen;cnt++)   EnvacIO.tx_data[len++] = *(msg_data+cnt);
  EnvacIO.tx_data[len++] =    Calc_Crc(EnvacIO.tx_data[POS_LEN]-2, EnvacIO.tx_data);
  EnvacIO.tx_data[len++] =    ETX; 

  return len;  
}


/**
  * @brief UART4 Initialization Function For RFID Interface
  * @param None
  * @retval None
  */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */
  // For RFID Interface
  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function For HOST Interface
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function For Envac Main Interface
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
}

uint8_t rep_comm_gate_open(void)
{
  uint8_t msg_data[1] = {0, };
  uint8_t len = makeBasePacket(HOST_CMD_REP_GATE_OPEN, 0, msg_data);
  uart_tx(&huart2, len, EnvacIO.tx_data);
  printf("\r\n send resp");
  return 0;
}


uint8_t rep_comm_bright(void)
{
  uint8_t msg_data[1] = {0, };

  msg_data[0] = EnvacIO.light_bright;
  uint8_t len = makeBasePacket(HOST_CMD_REP_LIGHT_BRIGHT, 1, msg_data);
  uart_tx(&huart2, len, EnvacIO.tx_data);
  printf("\r\n send resp");
  return 0;
}

uint8_t rep_comm_temerature(void)
{  
  char temperatureStr[5] ="";
  sprintf(temperatureStr, "%d", EnvacIO.temperature);
  uint8_t str_len = strlen(temperatureStr);
  uint8_t len = makeBasePacket(HOST_CMD_REP_TEMP,str_len, (uint8_t*)temperatureStr);

  uart_tx(&huart2, len, EnvacIO.tx_data);

  return 0;
}




uint8_t event_comm_detect_sensor(uint8_t sen_in)
{
  uint8_t msg_data[1] = {0x00, };
  
  if(sen_in) msg_data[0] = PIR_DETECT;  
  else       msg_data[1] = PIR_RELEASE;
  uint8_t len = makeBasePacket(HOST_CMD_EVENT_PIR_IN, 1, msg_data);
  uart_tx(&huart2, len, EnvacIO.tx_data);

  return 0;
}

uint8_t event_comm_rfid_in(void)
{
  uint8_t msg_data[20] = {0, };
  uint8_t len = makeBasePacket(HOST_CMD_EVENT_RFID_IN, 0, msg_data);
  uart_tx(&huart2, len, EnvacIO.tx_data);
  return 0;
}

uint8_t RfidParse_Packet(uint8_t data)
{
  //printf("\r\nrcvlen: %d", rfid_len);
  switch(rfid_rcv_mode)
  {
    case COMM_IDLE : 
    {
      if(data == STX) {
        rfid_rcv_mode = COMM_MSG;
        //printf("\r\nRFID Rx: [%02x] ", data);
        rfid_buff[rfid_len++] = data;
      }
      else {
        rfid_len =0;
      }
      break;
    }
    case COMM_MSG :
    {
      if(rfid_len < RFID_COMM_LEN-1) {
        rfid_buff[rfid_len++] = data;
        //printf("[%02x] ", data);
      }
      else {
        //printf("\r\nRECV END ");
        rfid_len =0;
        rfid_rcv_mode = COMM_IDLE;
        event_comm_rfid_in();
        //hex_dump(rfid_buff, 12, 20, "_");
        //printf("\r\n");
        //uint8_t gate_open_packet[11] = {0x02, 0x31, 0x30, 0x31, 0x32, 0x34, 0x35, 0x33, 0x37, 0x03, 0x36};
        //uart_tx(&huart3, 12, rfid_buff);
      }
      break;
    }
    default: 
    {
      rfid_len =0;
      rfid_rcv_mode = COMM_IDLE;
    }
  }
  return 0;
}

uint8_t Parse_Packet(uint8_t data) 
{
   switch(rcv_mode)
   {
        case COMM_IDLE :
            if(data == STX) {
                rcv_mode = COMM_LENGTH;
                printf("\r\nRx: [%02x] ", data);
            }
            break;

        case COMM_LENGTH :
            p_len += data;
            //p_len -= 4;// p_len :: msg size
            if(p_len > MAX_PACKET_SIZE) {   //max msg size 10
                printf("\r\n packet size over error");
                p_len =0;
                msg_len =0;
                rcv_mode = COMM_IDLE;
            }
            printf("[%02x] ", data);
            rcv_mode = COMM_CMD;
            break;

        case COMM_CMD : 
            rcv_cmd = data;
            rcv_mode = COMM_MSG;
            printf("[%02x] ", data);
            break;

        case COMM_MSG :
            if(msg_len < (p_len-4)) {
                rcv_buf[msg_len] = data;
                msg_len++;
                printf("[%02x] ", data);
                break;
            }
            else if(msg_len == (p_len-4)) {
                if(data == ETX) {
                  printf("[%02x] ", data);
                	proc_comm(rcv_cmd);
                }
                else {
                    printf("\r\n wrong etx packet.");
                }
                p_len =0;
                msg_len =0;
                rcv_mode = COMM_IDLE;
            }
            break;
   }
   return 0;
}

void read_usart(void)
{
  while(Uart2_Is_Empty() == 0) {
    Parse_Packet(Uart2_DeQueue());
  }
  while(Uart4_Is_Empty() == 0) {
    RfidParse_Packet(Uart4_DeQueue());
  }
}


uint8_t proc_comm(uint8_t cmd)
{
    switch(cmd)
    {
      case HOST_CMD_REQ_LIGHT_BRIGHT :
      {
        printf("\r\n cmd: COMM_REQ_LIGHT_BRIGHT[%02x][%02x]", rcv_cmd, rcv_buf[0]);
        if(SetLightBright(rcv_buf[0]) == 0) {
          rep_comm_bright();
        }
        break;
      }
      case HOST_CMD_REQ_GATE_OPEN :
      {
        printf("\r\n cmd: COMM_REQ_GATE_OPEN[%02x]", rcv_cmd);
        uint8_t gate_open_packet[11] = {0x02, 0x31, 0x30, 0x31, 0x32, 0x34, 0x35, 0x33, 0x37, 0x03, 0x36};
        uart_tx(&huart3, 11, gate_open_packet);
        rep_comm_gate_open();
        break;
      }
      case HOST_CMD_REQ_TEMP :
      {
        printf("\r\n cmd: COMM_REQ_TEMPERATURE[%02x]", rcv_cmd);
        rep_comm_gate_open();
        break;
      }
      default :
      {
        break;
      }
          
    }
    return 0;
}




