/*
 * uart_buffer.c
 *
 *  Created on: Dec 9, 2022
 *      Author: marijn
 */
#include <stdlib.h>
#include <string.h>
#include "uart_buffer.h"



uart_in_buffer_TypeDef* new_uart_ibuf(UART_HandleTypeDef* uart_handle, uint32_t size) {
	uart_in_buffer_TypeDef* handle = calloc(1, sizeof(uart_in_buffer_TypeDef));
	handle->buffer = calloc(size, 1);  // array of size len initialized to 0
	handle->uart_handle = uart_handle;
	handle->size = size;
	handle->read = size;  // dma starts writing from end
	handle->end = &uart_handle->hdmarx->Instance->NDTR;  // dma index variable
	HAL_UART_Receive_DMA(handle->uart_handle, handle->buffer, handle->size);  // start receiving data
	return handle;
}

void uart_ibuf_reset(uart_in_buffer_TypeDef* handle) {
	HAL_UART_DMAStop(handle->uart_handle);
	memset(handle->buffer, 0, handle->size);
	HAL_UART_Receive_DMA(handle->uart_handle, handle->buffer, handle->size);
}


void uart_ibuf_get_struct(void* data, uint32_t size, validator_fn_TypeDef validator_fn) {

}
