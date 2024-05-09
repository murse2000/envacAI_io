#include "project.h"
#include "console.h"
#include "usart.h"
#include <stdio.h>


uint8_t u1_rx_buffer[U1_BUFFER_SIZE] = {0x00, };
uint32_t  u1_rx_point_head = 0;
uint32_t  u1_rx_point_tail = 0;
uint8_t u1_rx_cnt =0;			// packet number
uint8_t inpd_len =0;
char inpd[50] = {0x00, };

uint8_t Uart1_Is_Empty(void)
{
	if(u1_rx_point_head == u1_rx_point_tail) 	return 1; 
	else 										return 0; 
}

void Uart1_EnQueue(uint8_t data)
{
	u1_rx_buffer[u1_rx_point_head] = data;
	u1_increase_point_value(&u1_rx_point_head);
}

uint8_t Uart1_DeQueue(void)
{
	uint8_t retVal = u1_rx_buffer[u1_rx_point_tail];
	u1_increase_point_value(&u1_rx_point_tail);
	
	return retVal;
}

void u1_increase_point_value(uint32_t * data_p)
{
	(* data_p)++;
	if(U1_BUFFER_SIZE == (* data_p)) {
		(* data_p) = 0;
	}
}

void Uart1Queue_Initialize(void)
{
	u1_rx_point_head = u1_rx_point_tail = 0;
}


void uart_putc(uint8_t dat, UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit(huart, &dat, sizeof(dat), 1);
}

void Console_ReadUart(void)
{
  while(Uart1_Is_Empty() != 1) {
    usart_console(Uart1_DeQueue());
  }
}

void usart_console(uint8_t data)
{
	unsigned char i=0;
	char *ptr = NULL;

	if((data == 0x08) && (inpd_len > 0)) {
		uart_putc(0x08, &huart1);	//backspace
		uart_putc(0x20, &huart1);	//space
		uart_putc(0x08, &huart1);	//backspace
		inpd_len= inpd_len -1;
		inpd[inpd_len] = '\0';
		return;
	}
	else if(data != 0x08) {
		uart_putc(data, &huart1);
	}
	if(data == 0x0d)	//receive enter
	{
		ptr = strtok(inpd," ");
		if(!strcmp(inpd, "help")) {
			printf("Supported Commands: \r\n");
			printf("help\r\n\
			<light> <value:0 to 100>\r\n\
			<version>\r\n\
			<clock>\r\n\
			<reset>\r\n");
		}
		else if(inpd_len < 1) {
			for(i=0;i<sizeof(inpd_len);i++) inpd[i] = 0;
			inpd_len =0;
		}
		else if(!strcmp(inpd, "version")) {
			printf("\r\nAuther : by Mr.CrazyOh %s %s",  __DATE__, __TIME__);
		}
		else if(!strcmp(inpd, "reset")) {
			HAL_NVIC_SystemReset();
		}
		else if(!strcmp(inpd, "clock")) {
			print_SystemClock();
		}
		else if(!strcmp(inpd, "door")) {
			uint8_t gate_open_packet[11] = {0x02, 0x31, 0x30, 0x31, 0x32, 0x34, 0x35, 0x33, 0x37, 0x03, 0x36};
        	uart_tx(&huart3, 11, gate_open_packet);
		}
		else if(!strcmp(inpd, "light")) {
			ptr = strtok(NULL, " ");
			int dimmer = atoi(ptr);
			if(dimmer != 0) {
				SetLightBright(dimmer);
			}
			else {
				printf("Invalid command\r\n");
				SetLightBright(0);
			}
		}
		else if(!strcmp(inpd, "temp")) {
			printf("\r\nTemperature : %d C", prevTemperatureLevel);
		}
		else {
			printf("Invalid command\r\n");
		}

		for(i=0;i<sizeof(inpd);i++)   inpd[i] = '\0';
		inpd_len =0;
		printf("\r\nbnct>> ");
	}
	else {
		if(data != 0x08) {
			inpd[inpd_len++] = data;
		}
	}
}

/**
 * @brief 
 * 
 * @version 0.1
 * @author Crazyoh (jihoon.oh@dongaeltek.co.kr)
 * @date 2023-05-25
 * @copyright Copyright (c) 2023
 */
void hex_dump(const void *src, size_t length, size_t line_size, char *prefix)
{
	int i = 0;
	const unsigned char *address = src;
	const unsigned char *line = address;
	unsigned char c;

	printf("%s | ", prefix);
	while (length-- > 0) {
		printf("%02X ", *address++);
		if (!(++i % line_size) || (length == 0 && i % line_size)) {
			if (length == 0) {
				while (i++ % line_size)
					printf("__ ");
			}
			printf(" |");
			while (line < address) {
				c = *line++;
				printf("%c", (c < 32 || c > 126) ? '.' : c);
			}
			printf("|\n");
			if (length > 0)
				printf("%s | ", prefix);
		}
	}
}


