/*
 * uart_buffer.c
 *
 *  Created on: Dec 9, 2022
 *      Author: marijn
 */
#include <stdlib.h>
#include <string.h>
#include "uart_buffer.h"



uart_ibuf* new_uart_ibuf(UART_HandleTypeDef* uart_handle, uint32_t size) {
	uart_ibuf* handle = (uart_ibuf*)calloc(1, sizeof(uart_ibuf));
	handle->buffer = calloc(size, 1);  // array of size len initialized to 0
	handle->uart_handle = uart_handle;
	handle->size = size;
	handle->read = size;  // dma starts writing from end
	handle->end = &uart_handle->hdmarx->Instance->NDTR;  // dma index variable
	HAL_UART_Receive_DMA(uart_handle, handle->buffer, size);  // start receiving data
	return handle;
}

void uart_ibuf_reset(uart_ibuf* handle) {
	*handle->end = 1024;
	handle->read = 1024;
	memset(handle->buffer, 0, handle->size);
}

uint8_t uart_ibuf_increment(uart_ibuf* handle) {
	if (handle->read > handle->size) { uart_ibuf_reset(handle); return 1; }  // somehow got invalid value
	if (handle->read == (*handle->end)) { return 1; }  // error align byte not found
	handle->read = handle->read ? (handle->read - 1) : handle->size;
	return 0;
}

uint8_t uart_ibuf_align(uart_ibuf* handle, uint8_t byte) {
	if (handle->read > handle->size) { uart_ibuf_reset(handle); return 1; }  // somehow got invalid value
	while (*(handle->buffer + handle->read) != byte) {
		if (uart_ibuf_increment(handle)) { return 1; }
	};
	return 0;
	//return uart_ibuf_increment(handle);  // this should return 0 because the last byte was 0xff
}

uint8_t uart_ibuf_read(uart_ibuf* handle, void* output, uint32_t size) {
	if (handle->read > handle->size) { uart_ibuf_reset(handle); return 1; }  // somehow got invalid value
	if ((*handle->end) > handle->read && handle->read < size) {  // data is broken along the buffer limits
		if ((1024 - (*handle->end)) + handle->read < size) { return 1; }  // not enough data
		uint32_t split_size = size - handle->read;
		if (handle->read) { memcpy(output + split_size, handle->buffer, handle->read); }  // only try memcpy if handle->read is a non zero value
		handle->read = (1024 - split_size);
		memcpy(output, handle->buffer + handle->read, split_size);
		return 0;
	}
	if (handle->read - (*handle->end) < size) { return 1; }  // not enough data
	handle->read -= size;
	memcpy(output, handle->buffer + handle->read, size);
	return 0;
}
