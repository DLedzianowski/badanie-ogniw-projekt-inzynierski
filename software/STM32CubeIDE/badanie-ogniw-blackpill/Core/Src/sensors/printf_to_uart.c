/*
 * printf_to_uart.c
 *
 *  Created on: Dec 27, 2024
 *      Author: dominik
 */
#include "sensors/printf_to_uart.h"

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 1000);
	return ch;
}


uint8_t serial_transmit(uint8_t* message)
{
	return CDC_Transmit_FS(message, strlen((const char*)message));
}


int _write(int fd, unsigned char *buf, int len)
{
	CDC_Transmit_FS(buf, len);
    //serial_transmit((uint8_t*)buf);
    return len;
}
