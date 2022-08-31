#include "math.hpp"
#include "message.hpp"



/* hidden */
bool uart_active = false;
bool SPI_active = false;
SPISettings SPI_setting;

/* shared */
// motor_instruction
void Msg::Structures::MCU_state::write() {
  uint16 crc = Check::crc_16(this, 16);
  _write_raw(this, 16);
  _write_raw(&crc, 2);
}

void Msg::Structures::MCU_instruction::set(Msg::Structures::motor_instruction* instruction) {
  this->steps = instruction->steps;
  this->pulse_delay = instruction->pulse_delay;
  // TODO: use the settings if possible OTHERWISE CHANGE CRC16 TO CRC32 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  this->settings.micro_step = 8;  // 256 microsteps (2 ^ microstep)
  this->settings._ = 0x000;  // set the rest of the bits to 0
  this->crc = Check::crc_16(this, 14);  // this is not used yet
}

bool Msg::Structures::motor_instruction::read() {
	_read_raw(this, 16);
	uint64 crc = this->crc;
	if (Check::error_correct(this, 12, crc, Check::CRC_TYPE::CRC_32)) {
		this->crc = crc & 0xfffffffful; return true;
	}
  this->steps = 0x0000000000000000ll;
  this->pulse_delay = 0x00000000ul;
  this->crc = 0x00000000ul; return false;
}

// header
void Msg::header::set(Msg::OP_CODE op_code, uint8 msg_id, uint8 motor_id, uint8 poly_data) {
	this->data.op_code = op_code; this->data.msg_id = msg_id;
	this->data.motor_id = motor_id; this->data.poly_data = poly_data;
	this->data.crc = Check::crc_16(this, 2);
}
bool Msg::header::load(uint32 value) {
	this->raw = value;
  uint64 crc = this->data.crc;
	if (Check::error_correct(this, 2, crc, Check::CRC_TYPE::CRC_16)) {
		this->data.crc = crc & 0xffff; return true;
	} return false;
}

// these are not header methods because data is simply loaded to an existing variable
bool Msg::read_header(Msg::header* const op)		    { uint32 buffer; Msg::_read_raw(&buffer, 4); return op->load(buffer); }
void Msg::write_header(const Msg::header* const op)	{ uint32 buffer = op->raw; Msg::_write_raw(&buffer, 4); }


// (uart functions)
void Msg::initialize_uart(const uint32 baudrate) {
	if (uart_active) { Serial.end(); }
	Serial.begin(baudrate);
	while (!Serial) { delay(1); }	// wait until the serial port is ready
	uart_active = true;
}
void Msg::initialize_SPI(const uint8 divide, const uint32 baudrate, const uint8 bit_order, const uint8 data_mode) {
	if (SPI_active) { SPI.end(); }
	SPI.begin();
  SPI.setClockDivider(divide);
  Msg::set_SPI_setting(baudrate, bit_order, data_mode);
	SPI_active = true;
}
void Msg::set_SPI_setting(const uint32 baudrate, const uint8 bit_order, const uint8 data_mode) { SPI_setting = SPISettings(baudrate, bit_order, data_mode); }
void Msg::start_SPI_transaction() { SPI.beginTransaction(SPI_setting); }
void Msg::end_SPI_transaction()   { SPI.endTransaction(); }
void Msg::SPI_tx_rx(const void* const tx, void* const rx, const uint64 bytes) { for (uint64 i = 0; i < bytes; i++) { ((uint8*)rx)[i] = SPI.transfer(((uint8*)tx)[i]); } }

void Msg::_read_raw(void* const buffer, const uint64 bytes) {
	while (Serial.available() < bytes) { delayMicroseconds(10);	/* wait for a tiny period until buffer has enough bytes to start reading */ }
	Serial.readBytes((int8*)buffer, bytes);
}
void Msg::_write_raw(const void* const buffer, const uint64 bytes) {
	while (!Serial.availableForWrite()) { delayMicroseconds(10);	/* wait for a tiny period until buffer is ready */ }
	Serial.write((int8*)buffer, bytes);
}