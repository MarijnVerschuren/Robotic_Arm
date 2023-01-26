/*
 * spi_buffer.h
 *
 *  Created on: Jan 24, 2023
 *      Author: marijn
 */

#ifndef INC_SPI_BUFFER_H_
#define INC_SPI_BUFFER_H_

#include "main.h"
#include "spi.h"


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
	uint8_t* buffer_end;
	SPI_HandleTypeDef* spi_handle;
	volatile uint32_t* write;  // where the uart peripheral is writing
	uint32_t read;
	uint32_t size;
} ibuf_TypeDef;

ibuf_TypeDef* new_ibuf(SPI_HandleTypeDef* spi_handle, uint32_t size);
void ibuf_reset(ibuf_TypeDef* handle);

uint8_t ibuf_buffered(ibuf_TypeDef* handle, uint32_t* size);
uint8_t ibuf_get(ibuf_TypeDef* handle, void* data, uint32_t size);
uint8_t ibuf_get_struct(ibuf_TypeDef* handle, void* data, uint32_t size, validator_fn_TypeDef validator_fn);


#endif /* INC_SPI_BUFFER_H_ */
