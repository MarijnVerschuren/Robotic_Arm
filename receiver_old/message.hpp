#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "macro.hpp"
#include "math.hpp"



namespace Msg {
  constexpr uint64 NULL_MSG = 0x0000000000000000ull;
  // structs / functions
	namespace Structures {
    // STM32 =[no crc included]=> ARD =[crc included]=> MASTER
    struct MCU_state {
      // this data is received from STM32 via SPI
      int64 pos;
      int64 job;

      // this facilitates the ARD => MASTER transmision via UART
      void write();  // generates a crc_16
    };

    // MASTER =[crc included]=> ARD =[no crc included]=> STM32
		struct motor_instruction {
      // the steps variable is sent to STM32 via SPI
			int64 steps;
      uint32 pulse_delay;  // delay from 1us -> 4295s (1 is added to delay because 0 us is not valid)
			uint32 crc;	// crc_32  (the crc is so large to fill up bits and to ensure correctness of the amount of steps)

      // this facilitates the MASTER => ARD transmision via UART
			bool read();
		};

    struct MCU_instruction {
      int64_t steps;
      uint32_t pulse_delay;  // delay from 1us -> 4295s (1 is added to delay because 0 us is not valid)
      struct {  // TODO: use the settings if possible OTHERWISE CHANGE CRC16 TO CRC32 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        uint8_t micro_step : 4;  // 0 - 8 calculate setting by using (2 ^ micro_step)
        uint16_t _ : 12;
      } settings;
      uint16_t crc;  // this is not used yet

      void set(Msg::Structures::motor_instruction* instruction);
    };

    // ARD =[crc included]=> MASTER 
		struct init_message {
			struct {
				uint8 id_0 : 1;
				uint8 id_1 : 1;
				uint8 id_2 : 1;
				uint8 id_3 : 1;
				uint8 id_4 : 1;
				uint8 id_5 : 1;
				uint8 id_6 : 1;
				uint8 id_7 : 1;
				uint8 id_8 : 1;
				uint8 id_9 : 1;
				uint8 id_10 : 1;
				uint8 id_11 : 1;
				uint8 id_12 : 1;
				uint8 id_13 : 1;
				uint8 id_14 : 1;
				uint8 id_15 : 1;
			} motor_ids;
			uint8 crc;	// crc_8
		};  // TODO <<<<<<<<<<<<<<<<<<<<<<<<<<<<< (NOT USED YET)
	};


	enum OP_CODE {
		/*	header structure:
			0000    0000    0000      0000
			opcode, msg_id, motor_id, crc_4
		*/
		// 0x0 is not used becuase the message 0x0000 will then be possible wich is too easy to accidentally happen
    MOTOR_CONFIG = 0x1,	// request motor config  // TODO check if it is posible to digitally set config if so then make another opcode to set the config <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		MOTOR_MOVE = 0x2,	// instruct motor (set job)

		MOTOR_POS = 0x3,	// motor pos and job (packet from mcu)

		ACK = 0xd,
		NACK = 0xe,

		INIT = 0xf			// start connection
	};

	union header {
		uint32 raw;
    struct {
      // short[0]
      uint16 op_code		: 3;  // 7 types of opcodes
      uint16 msg_id		  : 5;  // 32 messages ids
      uint16 motor_id		: 5;  // 32 motor ids
      uint16 poly_data	: 3;  // 0 - 7 datas after header
      // poly_data allows consecutive instructions for one motor
      // short[1]
      uint16 crc;  // crc_16
    } data;

		header() = default;
	  bool load(uint32 value);
		void set(Msg::OP_CODE op_code, uint8 msg_id, uint8 motor_id, uint8 poly_data = 1);
	};
	
	bool read_header(Msg::header* const op);
	void write_header(const Msg::header* const op);


	void initialize_uart(const uint32 baudrate = 9600);
	void initialize_SPI(const uint8 divide = SPI_CLOCK_DIV4, const uint32 baudrate = 2000000, const uint8 bit_order = MSBFIRST, const uint8 data_mode = SPI_MODE0);
  void set_SPI_setting(const uint32 baudrate, const uint8 bit_order, const uint8 data_mode);
  void start_SPI_transaction();
  void end_SPI_transaction();
  void SPI_tx_rx(const void* const tx, void* const rx, const uint64 bytes);
	/* raw read / write functions(not independent meant for independent use)
	 * they read and write full bytes to and from the serial buffer but mask any unwanted bits */
	void _read_raw(void* const buffer, const uint64 bytes);
	void _write_raw(const void* const buffer, const uint64 bytes);
};