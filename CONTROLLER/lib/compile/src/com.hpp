#pragma once
#include <cstdlib>

#include "int.hpp"



extern const uint16_t crc16_dnp_table[256];
extern uint16_t instrution_id_counter;


/* Flags
 * part of instruction that tells mcu what to do with the provided data
 */
// 0xf
enum ACTION_FLAGS : uint8_t {
	// exec move
	EXEC = 0x1,
	// abort move currently executing in the motor and replacing it with the attached move (if EXEC is set) (IF EXEC is not set the move will only be aborted)
	// if override is not set the move is put into a queue on the motor driver mcu
	OVERRIDE = 0x2,
	// make movement synced with the next
	// movements that should be synced will wait on the motor driver mcu untill the sync pin is set low
	SYNC = 0x4,
	// poll motor position (if SYNC is set the mcu will return with all other poll events at once)
	POLL = 0x8
};

enum RETURN_FLAGS : uint8_t {
	OK = 0x01,
	CRC_FIXED = 0x02,
	CRC_ERROR = 0x04,
	ERROR_FIXED = 0x08,  // fixed invalid instruction
	ERROR = 0x10
};

/* State
 * state of the MCU
 */
// 0xffffffffffffffff 0xffffffffffffffff 0xffffffff 0xffffffff 0xffff 0xffff 0xff 0xff 0xff 0xff
typedef struct {  // uint8_t[32]
	double			vel;
	double			acc;
	struct {  // +- 188,744,040 deg
		int32_t		rotation: 20;
		uint32_t	angle: 12;
	}				pos, target;
	uint16_t		raw_angle;
	uint16_t		instrution_id;		// instruction id which is currently being executed
	uint16_t		micro_step: 2;  	// microstep setting
	uint16_t		srd_mode: 1;		// srd mode on the motor controller
	uint16_t		queue_size: 13;		// allows up to 8192 instructions in queue
	// lots of parity for status
	uint16_t		status: 4;			// status codes
	uint16_t		n_status: 4;		// ~status
	uint16_t		status_parity: 1;	// parity for status
	uint16_t		id : 7;				// reserved until the main controller fills this in
} MCU_State;

/* Instruction
 * motor instruction (includes flags)
 */
// 0xffffffffffffffff 0xffffffffffffffff 0xffffffffffffffff ((0x3, 0x4, 0x78, 0xff80) => 0xffff) 0xffff, 0xffff, 0xffff
typedef struct {  // uint8_t[32]
	double		target;			// rad
	double		max_vel;		// rad / s
	double		max_acc;		// rad / s^2
	uint16_t	micro_step: 2;  // microstep setting
	uint16_t	srd_mode: 1;	// srd mode on the motor controller
	uint16_t	action: 4;		// look in ACTION enum for possible actions  // TODO: rethink and implement
	uint16_t	_: 2;
	uint16_t	id: 7;			// selected motor
	uint16_t	instrution_id;	// instruction id
	uint16_t	__;				// reserved uint8_t[2]
	uint16_t	crc;
} MCU_Instruction;


/* Handshake
 * this struct is used to establish a handshake between pc and mcu
 * it deliberatly has no error checking so that unstable connections are not accepted
 * to establish a successfull connection the received data is just sent back to the mcu with some flags set to preform init actions
 */
// 0xffffffff ((0xfe 0x1 0xff 0xffff) => 0xffffffff)
typedef struct {  // uint8_t[8] // all variables are bit fields to prevent padding
	uint64_t baud: 32;			// TODO: switch baud on the go
	uint64_t motor_count: 7;	// = max motor id (starts counting from 0)
	uint64_t init_0: 1;			// make mcu initialize the motors to their 0 position
	uint64_t _: 8;				// reserved because this struct is minimally 8 bytes
	uint64_t crc: 16;
} CTRL_Handshake;  // control handshake



void get_MCU_State_data(uint8_t* package, double* vel, double* acc, int32_t* pos_rotation, int32_t* target_rotation, uint16_t* pos_angle, uint16_t* target_angle, uint16_t* raw_angle, uint16_t* instrution_id, uint8_t* micro_step, uint8_t* srd_mode, uint8_t* id, uint8_t* queue_size, uint8_t* status);

uint8_t* new_MCU_Instruction(uint16_t id, uint8_t action, double target, double max_vel, double max_acc, uint8_t micro_step, uint8_t srd_mode, uint16_t* instrution_id);
void get_MCU_Instruction_data(uint8_t* package, double* target, double* max_vel, double* max_acc, uint8_t* micro_step, uint8_t* srd_mode, uint8_t* action, uint8_t* id, uint16_t* instrution_id, uint16_t* crc);

// sets the settings of received handshake
uint8_t* new_CTRL_Handshake(uint8_t motor_count, uint8_t init_0, uint32_t baud);
// init is type uint32_t to force size of buffer
void get_CTRL_Handshake_data(uint64_t package, uint8_t* motor_count, uint8_t* init_0, uint32_t* baud, uint16_t* crc);

// crc16_dnp was the most error resilient for messages under 130 bytes with a hamming distance of 7!!
uint16_t crc16_dnp(const void* buffer, uint64_t size);
