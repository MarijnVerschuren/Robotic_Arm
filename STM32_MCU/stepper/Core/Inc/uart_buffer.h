/*
 * uart_buffer.h
 *
 *  Created on: Dec 9, 2022
 *      Author: marijn
 */

#ifndef INC_UART_BUFFER_H_
#define INC_UART_BUFFER_H_

typedef struct {
	uint8_t* buffer;
	UART_HandleTypeDef* uart_handle;
	uint32_t* end;  // where the uart peripheral is writing
	uint32_t size;
	uint32_t read;
} uart_buffer;


uart_buffer* new_uart_buffer(UART_HandleTypeDef* uart_handle, uint32_t size) {
	uart_buffer* handle = (uart_buffer*)calloc(1, sizeof(uart_buffer));
	handle->buffer = malloc(size);
	handle->uart_handle = uart_handle;
	handle->size = size;
	handle->end = uart_handle;
	return handle;
}


#endif /* INC_UART_BUFFER_H_ */
