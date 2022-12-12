/*
 * uart_buffer.h
 *
 *  Created on: Dec 9, 2022
 *      Author: marijn
 */

#ifndef INC_UART_BUFFER_H_
#define INC_UART_BUFFER_H_

#include "main.h"
#include "usart.h"



typedef struct {
	uint8_t* buffer;
	UART_HandleTypeDef* uart_handle;
	__IO uint32_t* end;  // where the uart peripheral is writing
	uint32_t size;
	uint32_t read;
} uart_ibuf;


uart_ibuf* new_uart_ibuf(UART_HandleTypeDef* uart_handle, uint32_t size);
void uart_ibuf_reset(uart_ibuf* handle);

uint8_t uart_ibuf_increment(uart_ibuf* handle);
uint8_t uart_ibuf_align(uart_ibuf* handle, uint8_t byte);
uint8_t uart_ibuf_read(uart_ibuf* handle, void* output, uint32_t size);

#endif /* INC_UART_BUFFER_H_ */
