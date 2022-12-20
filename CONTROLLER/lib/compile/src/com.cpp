#include "com.hpp"



const uint16_t crc16_dnp_table[256] = {  // 512b
        0x0000, 0x5e36, 0xbc6c, 0xe25a, 0x78d9, 0x26ef, 0xc4b5, 0x9a83,
        0x89ff, 0xd7c9, 0x3593, 0x6ba5, 0xf126, 0xaf10, 0x4d4a, 0x137c,
        0x6bb2, 0x3584, 0xd7de, 0x89e8, 0x136b, 0x4d5d, 0xaf07, 0xf131,
        0xe24d, 0xbc7b, 0x5e21, 0x0017, 0x9a94, 0xc4a2, 0x26f8, 0x78ce,
        0xaf29, 0xf11f, 0x1345, 0x4d73, 0xd7f0, 0x89c6, 0x6b9c, 0x35aa,
        0x26d6, 0x78e0, 0x9aba, 0xc48c, 0x5e0f, 0x0039, 0xe263, 0xbc55,
        0xc49b, 0x9aad, 0x78f7, 0x26c1, 0xbc42, 0xe274, 0x002e, 0x5e18,
        0x4d64, 0x1352, 0xf108, 0xaf3e, 0x35bd, 0x6b8b, 0x89d1, 0xd7e7,
        0x5e53, 0x0065, 0xe23f, 0xbc09, 0x268a, 0x78bc, 0x9ae6, 0xc4d0,
        0xd7ac, 0x899a, 0x6bc0, 0x35f6, 0xaf75, 0xf143, 0x1319, 0x4d2f,
        0x35e1, 0x6bd7, 0x898d, 0xd7bb, 0x4d38, 0x130e, 0xf154, 0xaf62,
        0xbc1e, 0xe228, 0x0072, 0x5e44, 0xc4c7, 0x9af1, 0x78ab, 0x269d,
        0xf17a, 0xaf4c, 0x4d16, 0x1320, 0x89a3, 0xd795, 0x35cf, 0x6bf9,
        0x7885, 0x26b3, 0xc4e9, 0x9adf, 0x005c, 0x5e6a, 0xbc30, 0xe206,
        0x9ac8, 0xc4fe, 0x26a4, 0x7892, 0xe211, 0xbc27, 0x5e7d, 0x004b,
        0x1337, 0x4d01, 0xaf5b, 0xf16d, 0x6bee, 0x35d8, 0xd782, 0x89b4,
        0xbca6, 0xe290, 0x00ca, 0x5efc, 0xc47f, 0x9a49, 0x7813, 0x2625,
        0x3559, 0x6b6f, 0x8935, 0xd703, 0x4d80, 0x13b6, 0xf1ec, 0xafda,
        0xd714, 0x8922, 0x6b78, 0x354e, 0xafcd, 0xf1fb, 0x13a1, 0x4d97,
        0x5eeb, 0x00dd, 0xe287, 0xbcb1, 0x2632, 0x7804, 0x9a5e, 0xc468,
        0x138f, 0x4db9, 0xafe3, 0xf1d5, 0x6b56, 0x3560, 0xd73a, 0x890c,
        0x9a70, 0xc446, 0x261c, 0x782a, 0xe2a9, 0xbc9f, 0x5ec5, 0x00f3,
        0x783d, 0x260b, 0xc451, 0x9a67, 0x00e4, 0x5ed2, 0xbc88, 0xe2be,
        0xf1c2, 0xaff4, 0x4dae, 0x1398, 0x891b, 0xd72d, 0x3577, 0x6b41,
        0xe2f5, 0xbcc3, 0x5e99, 0x00af, 0x9a2c, 0xc41a, 0x2640, 0x7876,
        0x6b0a, 0x353c, 0xd766, 0x8950, 0x13d3, 0x4de5, 0xafbf, 0xf189,
        0x8947, 0xd771, 0x352b, 0x6b1d, 0xf19e, 0xafa8, 0x4df2, 0x13c4,
        0x00b8, 0x5e8e, 0xbcd4, 0xe2e2, 0x7861, 0x2657, 0xc40d, 0x9a3b,
        0x4ddc, 0x13ea, 0xf1b0, 0xaf86, 0x3505, 0x6b33, 0x8969, 0xd75f,
        0xc423, 0x9a15, 0x784f, 0x2679, 0xbcfa, 0xe2cc, 0x0096, 0x5ea0,
        0x266e, 0x7858, 0x9a02, 0xc434, 0x5eb7, 0x0081, 0xe2db, 0xbced,
        0xaf91, 0xf1a7, 0x13fd, 0x4dcb, 0xd748, 0x897e, 0x6b24, 0x3512
};

uint16_t instrution_id_counter = 0;



uint8_t* new_MCU_State(double vel, double acc, int32_t pos_rotation, int32_t target_rotation, uint16_t pos_angle, uint16_t target_angle, uint16_t raw_angle, uint16_t instrution_id, uint8_t micro_step, uint8_t srd_mode, uint8_t id, uint8_t queue_size, uint8_t queue_index) {
	MCU_State* data =	(MCU_State*)malloc(32);
	data->vel =		vel;
	data->acc =		acc;
	data->pos.rotation =	pos_rotation;
	data->target.rotation =	target_rotation;
	data->pos.angle =	pos_angle;
	data->target.angle =	target_angle;
	data->instrution_id =	instrution_id;
	data->micro_step =	micro_step;
	data->srd_mode =	srd_mode;
	data->id =		id;
	data->queue_size =	queue_size;
	data->queue_index =	queue_index;
	return (uint8_t*)data;
}

void get_MCU_State_data(uint8_t* package, double* vel, double* acc, int32_t* pos_rotation, int32_t* target_rotation, uint16_t* pos_angle, uint16_t* target_angle, uint16_t* raw_angle, uint16_t* instrution_id, uint8_t* micro_step, uint8_t* srd_mode, uint8_t* id, uint8_t* queue_size, uint8_t* queue_index) {
	MCU_State* data =	(MCU_State*)package;
	*vel =			data->vel;
	*acc =			data->acc;
	*pos_rotation =		data->pos.rotation;
	*target_rotation =	data->target.rotation;
	*pos_angle =		data->pos.angle;
	*target_angle =		data->target.angle;
	*raw_angle =		data->raw_angle;
	*instrution_id =	data->instrution_id;
	*micro_step =		data->micro_step;
	*srd_mode =		data->srd_mode;
	*id =			data->id;
	*queue_size =		data->queue_size;
	*queue_index =		data->queue_index;
}


uint8_t* new_MCU_Instruction(uint16_t id, uint8_t action, double target, double max_vel, double max_acc, uint8_t micro_step, uint8_t srd_mode, uint8_t dir, uint16_t* instrution_id) {
	MCU_Instruction* data =	(MCU_Instruction*)malloc(32);
	data->target =		target;
	data->max_vel =		max_vel;
	data->max_acc =		max_acc;
	data->micro_step =	micro_step;
	data->srd_mode =	srd_mode;
	data->action =		action;
	data->dir =		dir;
	data->id =		id;
	data->instrution_id =	instrution_id_counter;
	*instrution_id =	instrution_id_counter;
	instrution_id_counter++;
	data->crc = 		crc16_dnp(data, 30);
	return (uint8_t*)data;
}

void get_MCU_Instruction_data(uint8_t* package, double* target, double* max_vel, double* max_acc, uint8_t* micro_step, uint8_t* srd_mode, uint8_t* action, uint8_t* dir, uint8_t* id, uint16_t* instrution_id uint16_t* crc) {
	MCU_Instruction* data =	(MCU_Instruction*)package;
	*target =		data->target;
	*max_vel =		data->max_vel;
	*max_acc =		data->max_acc;
	*micro_step =		data->micro_step;
	*srd_mode =		data->srd_mode;
	*action =		data->action;
	*dir =			data->dir;
	*id =			data->id;
	*instrution_id =	data->instrution_id;
	*crc =			data->crc;
}


uint8_t* new_CTRL_Handshake(uint8_t motor_count, uint8_t init_0, uint32_t baud) {
	CTRL_Handshake* data =	(CTRL_Handshake*)malloc(6);
	data->motor_count =	motor_count;
	data->init_0 =		init_0;
	data->baud =		baud;
	data->crc = 		crc16_dnp(data, 4);
	return (uint8_t*)data;
}

void get_CTRL_Handshake_data(uint32_t package, uint8_t* motor_count, uint8_t* init_0, uint32_t* baud, uint16_t* crc) {
	CTRL_Handshake* data =	(CTRL_Handshake*)&package;
	// in this struct counting starts from 0
	*motor_count =		data->motor_count;
	*init_0 =		data->init_0;
	*baud =			data->baud;
	*crc =			data->crc;
}


// crc16_dnp was the most error resilient for messages under 130 bytes with a hamming distance of 7!!
uint16_t crc16_dnp(const void* buffer, uint64_t size) {
	uint16_t crc = 0x0000;
	for (uint64_t i = 0; i < size; i++) {
		crc = ((crc << 8) & 0xff00) ^ crc16_dnp_table[((crc >> 8) ^ ((const uint8_t*)buffer)[i]) & 0xff];
	}; return crc ^ 0xffff;
}