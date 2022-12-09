#include "com.hpp"



uint8_t* new_instruction(uint16_t id, uint8_t action, double target, double max_vel, double max_acc, uint8_t micro_step, uint8_t srd_mode) {
	instruction* data =		(instruction*)malloc(sizeof(instruction));
	data->target =			target;
	data->max_vel =			max_vel;
	data->max_acc =			max_acc;
	data->micro_step =		micro_step;
	data->srd_mode =		srd_mode;
	data->action =			action;
	data->id =				id;
	data->crc = 0xffff;  // TODO: crc
	return (uint8_t*)data;
}


uint8_t* new_handshake(uint8_t motor_count, uint8_t init_0, uint32_t baud) {
	handshake* data =	(handshake*)malloc(sizeof(handshake));
	data->motor_count =	motor_count;
	data->init_0 =		init_0;
	data->baud =		baud;
	data->crc = 0xffff;  // TODO: crc
	return (uint8_t*)data;
}


void get_handshake_data(uint32_t init, uint8_t* motor_count, uint8_t* init_0, uint32_t* baud, uint16_t* crc) {
	handshake* data =	(handshake*)&init;
	// in this struct counting starts from 0
	*motor_count =		data->motor_count;
	*init_0 =			data->init_0;
	*baud =				data->baud;
	*crc =				data->crc;
}