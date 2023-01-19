/*
 * uart_buffer.c
 *
 *  Created on: Dec 9, 2022
 *      Author: marijn
 */
#include <stdlib.h>
#include <string.h>
#include "uart_buffer.h"



ibuf_TypeDef* new_ibuf(UART_HandleTypeDef* uart_handle, uint32_t size) {
	ibuf_TypeDef* handle = calloc(1, sizeof(ibuf_TypeDef));
	handle->buffer = calloc(size, 1);  // array of size len initialized to 0
	handle->uart_handle = uart_handle;
	handle->size = size;
	handle->read = size;  // dma starts writing from end
	handle->end = &uart_handle->hdmarx->Instance->NDTR;  // dma index variable
	HAL_UART_Receive_DMA(handle->uart_handle, handle->buffer, handle->size);  // start receiving data
	return handle;
}

void ibuf_reset(ibuf_TypeDef* handle) {
	HAL_UART_DMAStop(handle->uart_handle);
	memset(handle->buffer, 0, handle->size);
	HAL_UART_Receive_DMA(handle->uart_handle, handle->buffer, handle->size);
}

uint8_t ibuf_buffered(ibuf_TypeDef* handle, uint32_t* size) {
	if ((*handle->end) < handle->read) {
		(*size) = (handle->size - handle->read) + (*handle->end);
		return RETURN_BUFFER_FRACTURE;
	}
	(*size) = (*handle->end) - handle->read;
	return RETURN_OK;
}

uint8_t ibuf_get(ibuf_TypeDef* handle, void* data, uint32_t size) {
	uint32_t buffered;
	uint8_t code = ibuf_buffered(handle, &buffered);
	if (buffered < size) { return code | RETURN_OUT_OF_DATA; }
	if (code && handle->read + size > handle->size) {
		uint32_t first_part = handle->size - handle->read;
		memcpy(data, handle->buffer + handle->read, first_part);
		memcpy(data + first_part, handle->buffer, size - first_part);
		handle->read = size - first_part; return code | RETURN_OK;
	}
	memcpy(data, handle->buffer + handle->read, size);
	handle->read += size; return code | RETURN_OK;
}
uint8_t ibuf_get_struct(ibuf_TypeDef* handle, void* data, uint32_t size, validator_fn_TypeDef validator_fn) {
	uint8_t code = ibuf_get(handle, data, size);
	if (code & RETURN_OUT_OF_DATA) { return code; }
	uint32_t overhead;
	code |= ibuf_buffered(handle, overhead);
	uint8_t valid = validator_fn(data);
	while (!valid) {
		if (!overhead) { return code | RETURN_STRUCT_INVALID; }
		memmove(data, data + 1, size - 1);
		((uint8_t*)data)[size - 1] = handle->buffer[handle->read];
		valid = validator_fn(data);
		handle->read++; overhead--;
	} return code | RETURN_OK | RETURN_STRUCT_VALID;
}
