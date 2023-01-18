/*
 * crc.h
 *
 *  Created on: Dec 11, 2022
 *      Author: marijn
 */

#ifndef INC_CRC_H_
#define INC_CRC_H_

#include "main.h"


extern const uint16_t crc16_dnp_table[256];

// crc16_dnp was the most error resilient for messages under 130 bytes with a hamming distance of 7!!
uint16_t crc16_dnp(const void* buffer, uint64_t size);


#endif /* INC_CRC_H_ */
