/*
 * printf_to_uart.h
 *
 *  Created on: Aug 11, 2025
 *      Author: idomi
 */

#ifndef INC_SENSORS_PRINTF_TO_UART_H_
#define INC_SENSORS_PRINTF_TO_UART_H_

// #include "usart.h"
#include "usbd_cdc_if.h"

// int __io_putchar(int ch);
uint8_t serial_transmit(uint8_t* message);
int _write(int fd, unsigned char *buf, int len);

#endif /* INC_SENSORS_PRINTF_TO_UART_H_ */
