import serial
import struct  # temp
import time
import sys

from lib import *



# init_args: (init_0, baud)
def init(adapter: str, *init_args, baud: int = 9600, time_out: int = 10) -> int:
	ser = serial.Serial(
		port=adapter, baudrate=baud, bytesize=serial.EIGHTBITS,
		parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_TWO,
		timeout=time_out
	)

	ser.reset_input_buffer()
	ser.reset_output_buffer()

	data_o = new_handshake(2, *init_args)
	print(''.join(format(x, '08b') for x in data_o), get_handshake_data(data_o), len(data_o))
	while True:
		ser.write(data_o)
		time.sleep(.1)
		if ser.inWaiting() < 6: continue  # no response
		data_i = ser.read(6)
		#data_i = data_i[1:6];
		data_if = get_handshake_data(data_i)
		print(''.join(format(x, '08b') for x in data_i), data_if, data_i)

		if data_if[1:3] == init_args and data_if[3] == 0xffff:
			if init_args[1] != baud: return init(adapter, *init_args, baud = init_args[1])
			return data_if[0]  # motor_count
		# TODO: add crc
		# TODO: when data is not received correctly reset robot arm



def run(adapter: str, baud: int = 9600, time_out: int = 10) -> None:
	motor_count = init(adapter, 1, baud)  # init args: init_0=1, baud=baud

	ser = serial.Serial(
		port=adapter, baudrate=baud, bytesize=serial.EIGHTBITS,
		parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
		timeout=time_out
	)

	print(ser.name)
	ser.reset_input_buffer()
	ser.reset_output_buffer()

	while True:
		ser.write(0xe5)
		length = ser.inWaiting()
		if length:
			data = ser.read(length)
			print(data)
            #for byte in data:
			#	print(format(byte, '#010b'), end=" ")
			#print(end="\n")
			#print(data.decode("utf-8"))
		time.sleep(.2)


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
