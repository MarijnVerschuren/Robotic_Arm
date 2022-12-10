#pragma once
#include <cstdlib>

#include "int.hpp"



#define SYNC_BYTE 0x5C



/* Flags
 * part of instruction that tells mcu what to do with the provided data
 */
// 0xf
enum flags : uint8_t {
	// exec move
	EXEC = 0x01,
	// abort move currently executing in the motor and replacing it with the attached move (if EXEC is set) (IF EXEC is not set the move will only be aborted)
	// if override is not set the move is put into a queue on the motor driver mcu
	OVERRIDE = 0x02,
	// make movement synced with the next
	// movements that should be synced will wait on the motor driver mcu untill the sync pin is set low
	SYNC = 0x04,
	// poll motor position (if SYNC is set the mcu will return with all other poll events at once)
	POLL = 0x08
};

/* Instruction
 * motor instruction (includes flags)
 */
// 0xffffffffffffffff 0xffffffffffffffff 0xffffffffffffffff ((0x3, 0x4, 0x78, 0xff80) => 0xffff) 0xffff
struct instruction {  // uint8_t[28]
	double		target;			// rad
	double		max_vel;		// rad / s
	double		max_acc;		// rad / s^2
	uint16_t	micro_step: 2;  // microstep setting
	uint16_t	srd_mode: 1;	// srd mode on the motor controller
	uint16_t	action: 4;		// look in ACTION enum for possible actions
	uint16_t	id: 5;			// selected motor
	uint16_t	crc;			// TODO (not a priority)
};

/* Handshake
 * this struct is used to establish a handshake between pc and mcu
 * it deliberatly has no error checking so that unstable connections are not accepted
 * to establish a successfull connection the received data is just sent back to the mcu with some flags set to preform init actions
 */
// ((0x1f, 0x20, 0xffffffc) => 0xffffffff, 0xffff
struct handshake {  // uint8_t[6]
	uint32_t motor_count: 5;	// = max motor id (starts counting from 0)
	uint32_t init_0: 1;			// make mcu initialize the motors to their 0 position
	// set baud rate after handshake
	// first handshake is always done with 9600 baud
	// after change handshake will have to be established again
	// https://community.st.com/s/question/0D53W00001GkPvRSAV/is-it-possible-to-change-the-baud-rate-of-the-usart-during-run-time-in-stm32-cube-ide
	uint32_t baud: 26;
	uint16_t crc;
};


// formats variables into a instruction packet
uint8_t* new_instruction(uint16_t id, uint8_t action, double target, double max_vel, double max_acc, uint8_t micro_step, uint8_t srd_mode);
void get_instruction_data(uint8_t* package, double* target, double* max_vel, double* max_acc, uint8_t* micro_step, uint8_t* srd_mode, uint8_t* action, uint8_t* id, uint16_t* crc);

// sets the settings of received handshake
uint8_t* new_handshake(uint8_t motor_count, uint8_t init_0, uint32_t baud);
// init is type uint32_t to force size of buffer
void get_handshake_data(uint32_t package, uint8_t* motor_count, uint8_t* init_0, uint32_t* baud, uint16_t* crc);