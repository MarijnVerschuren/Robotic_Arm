#include "message.hpp"
#include "math.hpp"


#define NCS_0_PIN 6
#define LED_PIN 10



/// <temp>
void pulse_led() {
	digitalWrite(LED_PIN, HIGH);
	delay(100);
	digitalWrite(LED_PIN, LOW);
	delay(100);
}
/// </temp>


	
void get_motor_config(uint8 motor_id);
Msg::Structures::MCU_state instruct_motor(uint8 motor_id);


Msg::header receive, send;
Msg::Structures::MCU_state state;

void setup() {
	pinMode(NCS_0_PIN, OUTPUT);
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(NCS_0_PIN, HIGH);	// set not_chip_select to high
	digitalWrite(LED_PIN, LOW);

	Msg::initialize_uart();
	Msg::initialize_SPI();

}

void loop() {
	/* tell the sender that a message came in broken
	 * (including the possibly broken data so that the sender might be able to identify the message and resend it) */
	if (!Msg::read_header(&receive)) {
		pulse_led();
		pulse_led();
		send.set(Msg::OP_CODE::NACK, receive.data.msg_id, receive.data.motor_id, 0);	// resend received message_id and motor_id in the hope that they remain unchanged (this is ran on crc correct fail)
		Msg::write_header(&send); return;
	}
	// CODE ERRORS OUT AND RECALLS LOOP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	switch (receive.data.op_code) {
	case Msg::OP_CODE::INIT:
		pulse_led();
		pulse_led();
		pulse_led();
		send.set(Msg::OP_CODE::ACK, receive.data.msg_id, receive.data.motor_id, 0);	// NO DATA YET
		Msg::write_header(&send);
		// todo send motor ids (with a promise that motors are connected from low to high id) (build config in master controller)
		break;
	case Msg::OP_CODE::MOTOR_CONFIG:
		pulse_led();
		pulse_led();
		pulse_led();
		pulse_led();
		get_motor_config(receive.data.motor_id);
		break;
	case Msg::OP_CODE::MOTOR_MOVE:	// set current job and return the MCU_state
		pulse_led();
		pulse_led();
		pulse_led();
		pulse_led();
		pulse_led();
		state = instruct_motor(receive.data.motor_id);
		send.set(Msg::OP_CODE::MOTOR_POS, receive.data.msg_id, receive.data.motor_id);
		Msg::write_header(&send);
		state.write(); break;
	default:
		pulse_led();
		pulse_led();
		pulse_led();
		pulse_led();
		pulse_led();
		pulse_led();
	}
}



void get_motor_config(uint8 motor_id) {
	return;	// this will send the motor config like microstep config etc..
}


/* <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	combine motor_pos and motor_instruct because the STM32 sends its state when receiving an instruction
	TODO:
		use the instruct opcode as input and pos opcode as an output
		remove the seperate switch case and functions
		
		add the step delay to the config that is sent to the controller so that the controller can estimate the time of completion
*/

Msg::Structures::MCU_state instruct_motor(uint8 motor_id) {
	Msg::Structures::motor_instruction instruction;
	if (!instruction.read()) {
		send.set(Msg::OP_CODE::NACK, receive.data.msg_id, receive.data.motor_id);
		Msg::write_header(&send); return;	// return to loop so that the controller can resend the command
	}

	Msg::Structures::MCU_state rx;
	Msg::Structures::MCU_instruction tx;
	tx.set(&instruction);

	Msg::start_SPI_transaction();
	digitalWrite(SS, LOW);
	Msg::SPI_tx_rx(&tx, &rx, 16);	// redo this function to include the start transaction and chip select code (for the correct chip)
	digitalWrite(SS, HIGH);
	Msg::end_SPI_transaction();
	
	return rx;
}
