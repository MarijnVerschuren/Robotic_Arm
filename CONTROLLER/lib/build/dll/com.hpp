#pragma once
#include <cstdlib>

#include "int.hpp"


// 0xffffffffffffffff 0xffffffff 0xffffffff 0xffffffff {0x3 0x4 0xfffffff8} => 0xffffffff
struct instruction {  // 24
	int64_t		target;
	uint32_t	min_delay;  // in us
	uint32_t	max_delay;  // in us
	float		dropoff_rate;
	uint32_t	micro_step: 2;
	uint32_t	srd_mode: 1;
	uint32_t	crc: 28;	// TODO (not a priority)
	uint32_t	parity: 1;  // for (2n -1) multi bit (n >= 2) errors
};


extern "C" __declspec(dllexport) char* format(int64_t target, uint32_t min_delay, uint32_t max_delay, float dropoff_rate, uint8_t micro_step, uint8_t srd_mode);
extern "C" __declspec(dllexport) void discard(char* data);