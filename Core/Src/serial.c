/*
 * serial.c
 *
 *  Created on: 9/09/2022
 *      Author: charles
 */

#include "serial.h"
#include "main.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

#include <stdio.h>
#include <string.h>

int serial_send(const uint8_t *buffer, int buffer_length)
{
	return HAL_UART_Transmit(&huart1, (uint8_t *)buffer, buffer_length, 10);
}

int serial_send_str(const char *str)
{
	return serial_send((const uint8_t *)str, strlen(str));
}
