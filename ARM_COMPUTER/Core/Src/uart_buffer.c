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
	handle->buffer_end = handle->buffer + size;
	handle->uart_handle = uart_handle;
	handle->write = &uart_handle->hdmarx->Instance->NDTR;  // dma index variable (indexes from end of buffer)
	handle->read = 0;
	handle->size = size;
	HAL_UART_Receive_DMA(handle->uart_handle, handle->buffer, size);  // start receiving data
	return handle;
}

void ibuf_reset(ibuf_TypeDef* handle) {
	HAL_UART_DMAStop(handle->uart_handle);
	memset(handle->buffer, 0, handle->size);
	HAL_UART_Receive_DMA(handle->uart_handle, handle->buffer, handle->size);
}

uint8_t ibuf_buffered(ibuf_TypeDef* handle, uint32_t* size) {
	uint32_t write = (handle->size - (*handle->write));
	if (write < handle->read) {
		(*size) = (handle->size - handle->read) + write;
		return RETURN_BUFFER_FRACTURE;
	}
	(*size) = write - handle->read;
	return RETURN_OK;
}

uint8_t ibuf_get(ibuf_TypeDef* handle, void* data, uint32_t size) {
	uint32_t buffered;
	uint8_t code = ibuf_buffered(handle, &buffered);
	if (buffered < size) { return code | RETURN_OUT_OF_DATA; }
	uint32_t part_size = (handle->size - handle->read);
	if ((code | RETURN_BUFFER_FRACTURE) && part_size < size) {
		memcpy(data, handle->buffer + handle->read, part_size);
		data += part_size;
		part_size = size - part_size;
		memcpy(data, handle->buffer, part_size);
		handle->read = handle->buffer + part_size;
		return code | RETURN_OK;
	}
	memcpy(data, handle->buffer + handle->read, size);
	handle->read += size;
	return code | RETURN_OK;
}
uint8_t ibuf_get_struct(ibuf_TypeDef* handle, void* data, uint32_t size, validator_fn_TypeDef validator_fn) {
	uint8_t code = ibuf_get(handle, data, size);
	if (code & RETURN_OUT_OF_DATA) { return code; }
	uint32_t overhead;
	code |= ibuf_buffered(handle, &overhead);
	uint8_t valid = validator_fn(data);
	while (!valid) {
		if (!overhead) { return code | RETURN_STRUCT_INVALID; }
		memmove(data, data + 1, size - 1);
		((uint8_t*)data)[size - 1] = *(handle->buffer + handle->read);
		valid = validator_fn(data);
		handle->read = (handle->read + 1) % handle->size;
		overhead--;
	} return code | RETURN_OK | RETURN_STRUCT_VALID;
}
