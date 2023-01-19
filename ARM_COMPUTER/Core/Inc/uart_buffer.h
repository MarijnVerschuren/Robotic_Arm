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

typedef enum {
	// OK codes
	RETURN_OK =					0x1,
	RETURN_BUFFER_FRACTURE =	0x2,	// data split over buffer edges
	RETURN_STRUCT_VALID =		0x4,
	// ERROR codes
	RETURN_OUT_OF_DATA =		0x10,	// not enough data in buffer
	RETURN_STRUCT_INVALID =		0x20	// struct data returned has not been validated
} buf_return_TypeDef;

typedef struct {
	uint8_t* buffer;
	UART_HandleTypeDef* uart_handle;
	volatile uint32_t* end;  // where the uart peripheral is writing
	uint32_t size;
	uint32_t read;
} ibuf_TypeDef;


ibuf_TypeDef* new_ibuf(UART_HandleTypeDef* uart_handle, uint32_t size);
void ibuf_reset(ibuf_TypeDef* handle);

uint8_t ibuf_buffered(ibuf_TypeDef* handle, uint32_t* size);
uint8_t ibuf_get(ibuf_TypeDef* handle, void* data, uint32_t size);
uint8_t ibuf_get_struct(ibuf_TypeDef* handle, void* data, uint32_t size, validator_fn_TypeDef validator_fn);


#endif /* INC_UART_BUFFER_H_ */
