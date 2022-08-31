#pragma once

#define int8	char
#define uint8	unsigned char
#define int16	short
#define uint16	unsigned short
#define int32	long
#define uint32	unsigned long
#define int64	long long
#define uint64	unsigned long long

#define uint	unsigned int

#define f32		float
#define f64		double



struct OP_CODE {
	/*	header structure:
		0000    0000    0000      0000
		opcode, msg_id, motor_id, crc_4
	*/
	enum _ {
		// 0x0 is not used becuase the message 0x0000 will then be possible wich is too easy to accidentally happen
		MOTOR_CONFIG = 0x1,	// request motor config
		MOTOR_MOVE = 0x2,	// instruct motor (set job)

		MOTOR_POS = 0x3,	// motor pos and job (packet from mcu)

		ACK = 0xd,
		NACK = 0xe,

		INIT = 0xf			// start connection
	};
};

struct CRC_TYPE {
	enum _ {
		CRC_4 =		0x0,	// 1/2 byte
		CRC_8 =		0x1,
		CRC_16 =	0x2,
		CRC_32 =	0x4,
		CRC_64 =	0x8
	};
};


const uint64	crc_64			(const int8* buffer, uint64 bytes);
const uint32	crc_32			(const int8* buffer, uint64 bytes);
const uint16	crc_16			(const int8* buffer, uint64 bytes);
const uint8		crc_8			(const int8* buffer, uint64 bytes);
const uint8		crc_4			(const int8* buffer, uint64 bytes);
bool 			error_correct	(int8* buffer, uint64 bytes, uint64& crc, uint8 type, uint64 bit_cutoff = 0);


union header {
	uint32 _raw;
	struct {
		// short[0]
		uint16 op_code		: 3;  // 7 types of opcodes
		uint16 msg_id		: 5;  // 32 messages ids
		uint16 motor_id		: 5;  // 32 motor ids
		uint16 poly_data	: 3;  // 0 - 7 datas after header
		// poly_data allows consecutive instructions for one motor
		// short[1]
		uint16 crc;  // crc_16
	} data;

	header() = default;
	bool	load(uint32 value);
	void	set(uint8 op_code, uint8 msg_id, uint8 motor_id, uint8 poly_data = 1);
	// for python swig compatibility
	constexpr inline uint8	get_op_code()	{ return data.op_code; }
	constexpr inline uint8	get_msg_id()	{ return data.msg_id; }
	constexpr inline uint8	get_motor_id()	{ return data.motor_id; }
	constexpr inline uint8	get_poly_data()	{ return data.poly_data; }
	constexpr inline uint16	get_crc()		{ return data.crc; }
};


struct motor_instruction {
	int64 steps;
    uint32 pulse_delay;  // delay from 1us -> 4295s (1 is added to delay because 0 us is not valid)
	uint32 crc;	// crc_32  (the crc is so large to fill up bits and to ensure correctness of the amount of steps)

	void set(int64 steps, uint32 pulse_delay);
};


struct MCU_state {
	int64 pos;
	int64 job;
	uint16 crc;

	bool load(const int8* buffer);
};