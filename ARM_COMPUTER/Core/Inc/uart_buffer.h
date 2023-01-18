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


typedef uint8_t (*validator_fn_TypeDef) (void*);


typedef struct {
	uint8_t* buffer;
	UART_HandleTypeDef* uart_handle;
	__IO uint32_t* end;  // where the uart peripheral is writing
	uint32_t size;
	uint32_t read;
} uart_in_buffer_TypeDef;


uart_in_buffer_TypeDef* new_uart_ibuf(UART_HandleTypeDef* uart_handle, uint32_t size);
void uart_ibuf_reset(uart_in_buffer_TypeDef* handle);


// get struct from buffer
void uart_ibuf_get_struct(void* data, uint32_t size, validator_fn_TypeDef validator_fn);


#endif /* INC_UART_BUFFER_H_ */
