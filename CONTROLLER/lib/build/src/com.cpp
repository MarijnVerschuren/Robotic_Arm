#include "com.hpp"



char* format(int64_t target, uint32_t min_delay, uint32_t max_delay, float dropoff_rate, uint8_t micro_step, uint8_t srd_mode) {
	instruction* data =		(instruction*)malloc(sizeof(instruction));
	data->target =			target;
	data->min_delay =		min_delay;
	data->max_delay =		max_delay;
	data->dropoff_rate =	dropoff_rate;
	data->micro_step =		micro_step;
	data->srd_mode =		srd_mode;
	return (char*)data;
}
