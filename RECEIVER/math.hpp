#pragma once

#include <Arduino.h>

#include "macro.hpp"



// #define ceil(x) (((int)(x)) == ((float)(x))) ? ((int)(x)) : (((int)(x)) + 1)
// #define floor(x) ((int)(x))

namespace Util {
	const uint64 strlen(str string);
};

namespace Check {
	typedef const uint64(*crc_functype) (const void* const, uint64);
	enum CRC_TYPE {
		CRC_4 =		0x0,	// 1/2 byte
		CRC_8 =		0x1,
		CRC_16 =	0x2,
		CRC_32 =	0x4,
		CRC_64 =	0x8
	};

	// TODO: test if bit shift in crc_8 is needed
	const uint64			crc_64	(const void* const buffer, uint64 bytes);
	const uint32			crc_32	(const void* const buffer, uint64 bytes);
	const uint16			crc_16	(const void* const buffer, uint64 bytes);
	const uint8				crc_8 (const void* const buffer, uint64 bytes);
	const uint8				crc_4	(const void* const buffer, uint64 bytes);

	/* this function alters the given buffer and crc to fix errors(true if fixed, false if not)
	 * it assumes that the buffer is "aligned" and only has data bits
	 * (look at the Buffer::align function description for more info)
	 * the last optional argument tells the funtion when the message in the buffer ends and the padding begins
	 * this can be useful in the case where a bit flip in the padding results in a false error correction
	 * the function may calculate the crc of the message with a bit fipped in the padding (which is not considered data)
	 * that is the same as the original crc (this will then count as corrected) */
	bool error_correct(void* const buffer, uint64 bytes, uint64& crc, CRC_TYPE type, uint64 bit_cutoff = 0);
};

namespace Math {
  inline uint16 byteswap_16(uint16 x) {
      return (x << 8) | (x >> 8);
  }
  inline int16 byteswap_16(int16 x) {
      return (x << 8) | ((x >> 8) & 0xFF);
  }
  inline uint32 byteswap_32(uint32 x) {
      x = ((x << 8) & 0xFF00FF00) | ((x >> 8) & 0xFF00FF);
      return (x << 16) | (x >> 16);
  }
  inline int32 byteswap_32(int32 x) {
      x = ((x << 8) & 0xFF00FF00) | ((x >> 8) & 0xFF00FF);
      return (x << 16) | ((x >> 16) & 0xFFFF);
  }
  inline uint64 byteswap_64(uint64 x) {
      x = ((x << 8) & 0xFF00FF00FF00FF00ULL) | ((x >> 8) & 0x00FF00FF00FF00FFULL);
      x = ((x << 16) & 0xFFFF0000FFFF0000ULL) | ((x >> 16) & 0x0000FFFF0000FFFFULL);
      return (x << 32) | (x >> 32);
  }
  inline int64 byteswap_64(int64 x) {
      x = ((x << 8) & 0xFF00FF00FF00FF00ULL) | ((x >> 8) & 0x00FF00FF00FF00FFULL);
      x = ((x << 16) & 0xFFFF0000FFFF0000ULL) | ((x >> 16) & 0x0000FFFF0000FFFFULL);
      return (x << 32) | ((x >> 32) & 0xFFFFFFFFULL);
  }
};