import serial
import struct  # temp
import time
import sys

from lib import *



# init_args: (init_0, baud)
def init(adapter: str, *init_args, baud: int = 9600, time_out: float = 0.1) -> int:
	ser = serial.Serial(
		port=adapter, baudrate=baud, bytesize=serial.EIGHTBITS,
		parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_TWO,
		timeout=time_out
	)

	ser.reset_input_buffer()
	ser.reset_output_buffer()
	
	data_o = new_CTRL_Handshake(0, *init_args)
	data_crc = get_CTRL_Handshake_data(data_o)[3]

	while True:
		ser.write(data_o)
		if ser.inWaiting() < 8: time.sleep(0.2); continue  # no response
		raw_i = ser.read(8)
		data_i = get_CTRL_Handshake_data(raw_i)
		print(data_i, raw_i)
		if data_i[1:3] == init_args:  # todo: check crc
			if init_args[1] != baud: return init(adapter, *init_args, baud = init_args[1])
			return data_i[0]  # motor_count



def run(adapter: str, baud: int = 9600, time_out: float = 0.1) -> None:
	motor_count = init(adapter, 1, baud)  # init args: init_0=1, baud=baud

	ser = serial.Serial(
		port=adapter, baudrate=baud, bytesize=serial.EIGHTBITS,
		parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_TWO,
		timeout=time_out
	)

	print(ser.name)
	ser.reset_input_buffer()
	ser.reset_output_buffer()

	while True:
		input("send: ")
		# make instruction
		target = float(input("target (f64, rad): "))
		max_vel = 10
		max_acc = 1
		micro_step = 3
		srd_mode = 0
		action = 0xf
		motor_id = 0#int(input(f"motor_id (0 to {motor_count}): "))
		instruction, instruction_id = new_MCU_Instruction(motor_id, action, target, max_vel, max_acc, micro_step, srd_mode)
		print(instruction, instruction_id)
		
		return_code = 0
		while (not return_code & RETURN_FLAGS.OK):
			ser.write(instruction)

			return_code = int.from_bytes(ser.read(1), "little")
			print(return_code)




if __name__ == "__main__":
	if "-help" in sys.argv:
		print(
			"* => required\n",
			"[-help]\t\t\t\tdisplay this message",
			"[-adapter](adapter)*\tset adapter to use",
			"[-baud](rate)\t\t\t\tset adapter baud rate",
			"[-time_out](time)\t\t\t\tset adapter baud rate",
			sep="\n", end="\n\n"
		)
		exit(0)
	if "-adapter" not in sys.argv: print("[-adaper] is a required argument"); exit(1)
	args =			{"adapter":		sys.argv[sys.argv.index("-adapter") + 1]}
	if "-baud" in sys.argv:
		args.update({"baud":		int(sys.argv[sys.argv.index("-baud") + 1])})
	if "-time_out" in sys.argv:
		args.update({"time_out":	float(sys.argv[sys.argv.index("-time_out") + 1])})

	"""
	data = new_instruction(1, flags.EXEC | flags.OVERRIDE | flags.SYNC | flags.POLL, 1.2, 32.3, 51.2, 2, 1)
	print(data[0:8].hex())
	print(data[8:16].hex())
	print(data[16:24].hex())
	print(data[24] & 0x3)
	print((data[24] & 0x4) >> 2)
	print((data[24] & 0x78) >> 3)
	print((int.from_bytes(data[24:26], "little") & 0xff80) >> 7)
	print(data[26:28])
	"""

	run(**args)

# sudo cat /proc/tty/driver/serial

# TODO: REVERSE INIT PROCESS

