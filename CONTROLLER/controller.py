import serial
import struct  # temp
import time
import sys


def run(adapter: str, baud: int = 115200, time_out: float = 100) -> None:
	ser = serial.Serial(
		port=adapter, baudrate=baud, bytesize=serial.SEVENBITS,
		parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
		timeout=time_out, xonxoff=False, rtscts=False,
		write_timeout=None, dsrdtr=False, inter_byte_timeout=None,
		exclusive=None
	)
	print(ser.name)
	ser.send_break(2)
	ser.reset_input_buffer()
	while True:
		length = ser.inWaiting()
		if length:
			data = ser.read(length)
			print(f"\n========== {time.time()}")
			try: print(data.decode("utf-8"))
			except: print(data)
			print("==========\n")
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

	run(**args)

# sudo cat /proc/tty/driver/serial