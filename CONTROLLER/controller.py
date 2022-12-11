import serial
import struct  # temp
import time
import sys

from lib import *



# init_args: (init_0, baud)
def init(adapter: str, *init_args, baud: int = 9600, time_out: float = 10) -> int:
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
		ser.write(bytes([SYNC_BYTE]))  # sync byte
		ser.write(data_o)
		if ser.inWaiting() < 6: continue  # no response
		data_i = get_CTRL_Handshake_data(ser.read(6))
		if data_i[1:3] == init_args and data_i[3] == data_crc:
			if init_args[1] != baud: return init(adapter, *init_args, baud = init_args[1])
			return data_i[0]  # motor_count
		# TODO: add crc on both ends



def run(adapter: str, baud: int = 9600, time_out: float = 10) -> None:
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
		""" # make instruction
		target = float(input("target (f64, rad): "))
		max_vel = float(input("max_vel (f64, rad / s): "))
		max_acc = float(input("max_acc (f64, rad / s^2): "))
		print(
			"microstep:",
			"\t(0) -> MS2",
			"\t(1) -> MS4",
			"\t(2) -> MS8",
			"\t(3) -> MS16",
			sep="\n"
		)
		micro_step = int(input(": "))
		srd_mode = int(input("srd_mode (1 or 0): "))
		print(
			"action (+ to use multiple):",
			"\t(1) -> EXEC",
			"\t(2) -> OVERRIDE",
			"\t(4) -> SYNC",
			"\t(8) -> POLL",
			sep="\n"
		)
		action = int(input(": "))
		motor_id = int(input(f"motor_id (0 to {motor_count}): "))
		instruction = new_MCU_Instruction(motor_id, action, target, max_vel, max_acc, micro_step, srd_mode)
		print(instruction)
		"""

		instruction = b"\x9a\x99\x99\x99\x99\xf9\x8f@ffffff$@333333\x0b@[\x00\x0e\x83"
		ser.write(bytes([SYNC_BYTE]))  # sync byte
		ser.write(instruction)

		time.sleep(.2)

		in_len = ser.inWaiting()
		if in_len > 27:
			print(instruction.hex(), ser.read(in_len).hex())




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
