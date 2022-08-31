from threading import Thread
from bitfield import make_bf as mkbf
from serial import Serial
import ctypes
import struct
import time
import sys

from lib import *


# NOTE: DONT WORRY ABOUT SERIAL HANGING BECAUSE THE REST OF THE MCUS WILL HANG ANYWAYS SO THE MASTER COUNTINUEING DOESTN HELP

class thread(Thread):
	def __init__(self, *args, **kwargs) -> None:
		super(thread, self).__init__(*args, **kwargs)

	def stop(self) -> bool:
		if not self.is_alive(): return True
		res = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(self.ident), ctypes.py_object(SystemExit))
		if res > 1:
			ctypes.pythonapi.PyThreadState_SetAsyncExc(self.ident, None)
			return False
		return True



class message:
	def __init__(self, op_code: int = 0, motor_id: int = 0, poly_data: int = 0, data: list = None) -> None:
		self.header = header()					# message header
		self.data = data if data else []		# data that is sent after the header

		self.op_code = op_code
		self.motor_id = motor_id
		self.poly_data = poly_data
		# threading lock
		self.lock = False

	def set(self, op_code: int, motor_id: int, poly_data: int, data: list = None) -> None:
		self.data = data if data else []		# data that is sent after the header
		self.op_code = op_code
		self.motor_id = motor_id
		self.poly_data = poly_data

	def send(self, serial, msg_id: int) -> None:
		self.header.set(self.op_code, msg_id, self.motor_id, self.poly_data)
		serial.write(self.header.raw)
		if not self.data: return
		serial.write(self.data.raw)

	@staticmethod
	def wait_for_bytes(serial, byte_count: int, timeout: int = 10) -> bool:
		while serial.in_waiting < byte_count:
			if timeout < 0: break
			time.sleep(0.1); timeout -= 0.1
		else: return True  # bytes in buffer
		return False  # timeout termination


	# CHECK IF THIS NEEDS THREADING <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	def receive(self, serial, halt: bool = True) -> thread or bool:  # returns a threading object or bool [False if receive fail] (threading object will evaluate to bool)
		# the halt arg will invoke a thread if False otherwise it will wait until received
		if not halt:
			if self.lock: False  # cant use threading because an other thread is busy
			self.lock = True
			t = thread(target=self.receive, args=(serial, ))
			t.start()
			return t
		if not self.wait_for_bytes(serial, 4): return False
		self.header.load(bytes_to_int(serial.read(4)))
		self.poly_data = self.header.get_poly_data()
		self.op_code = self.header.get_op_code()
		for i in range(self.poly_data):
			if self.op_code == OP_CODE.MOTOR_CONFIG:
				pass
			elif self.op_code == OP_CODE.MOTOR_POS:
				state = MCU_state()
				if not self.wait_for_bytes(serial, MCU_state.data_len): return False
				state.load(serial.read(MCU_state.data_len))
				self.data[i] = state.copy()  # copy so it can change and be deleted
			elif self.op_code == OP_CODE.ACK:
				pass
			elif self.op_code == OP_CODE.NACK:
				pass
			elif self.op_code == OP_CODE.INIT:
				pass


	def __str__(self) -> str:
		return f"""{{
			header: {self.header},
			data: {self.data}
		}}"""

	def __repr__(self) -> str: return str(self)



class message_box:
	def __init__(self, serial) -> None:
		self.serial = serial
		self.sent = [None] * 32  # sent messages
		
		self.inbox = []
		# threading lock
		self.lock = False


	def send(self, msg: message) -> bool:
		for index, sent in enumerate(self.sent):
			if not sent: break
		else: return False  # no space for message the caller has to hold the message
		msg.send(self.serial, index)		# set message id and calculate crc
		self.sent[index] = msg  # save to message box until confirmation is received
		return True

	def receive(self, msg: message) -> message or None:
		# if this returns None then the sent message with that id doesnt exist
		# that means that it has already received a previous message with this id
		# or that a previous message was received in a corrupted state and made it past the crc without fix
		index = msg.header.get_msg_id()
		sent_msg = self.sent[index]
		self.sent[index] = None  # free up this id in the message box
		return sent_msg  # return the original message (with id and crc) to the caller (be carefull for the issues listed above)


	def wait_for(self, serial, condition: callable, timeout: int = 60, halt: bool = True) -> thread or bool:  # returns a threading object or bool [False if receive fail] (threading object will evaluate to bool)
		# the halt arg will invoke a thread if False otherwise it will wait until received
		if not halt:
			if self.lock: False  # cant use threading because an other thread is busy
			self.lock = True
			t = thread(target=self.wait_for, args=(serial, condition, timeout))
			t.start()
			return t
		while timeout > 0:
			t1 = time.time()
			msg = message()
			msg.receive(serial)
			self.receive(msg)
			self.inbox.append(msg)
			if condition(msg): return True
			timeout -= (time.time() - t1)
		else: return False

	def __iter__(self) -> object: return self
	def __next__(self) -> message:
		if len(self.inbox) == 0: raise StopIteration
		msg = self.inbox[0]
		self.inbox.pop(0)
		return msg

	def __str__(self) -> str: return str(self.sent)




# wait 1 seconds for incomming data by default
def main(adapter: str, baudRate: int = 9600, timeout: int = 1) -> None:
	serial = Serial(adapter, baudRate, timeout=timeout)
	box = message_box(serial)
	rx = message()
	tx = message()


	# init step just tests the connection
	tx.set(OP_CODE.INIT, 0x0, 0)
	if not box.send(tx):  # TODO: SAVE TIME SENT <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		raise Exception("unable to send message!")
	box.wait_for(serial, lambda x: x.header.get_op_code() == OP_CODE.INIT, 4)
	for msg in box:
		print(msg)

	input("breakpoint")
	# STOP CODING HERE <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	# ARDUINO NEEDS TO UNDERSTAND HEADER STRUCTURE NOW <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	"""while True:
							serial.write(send_header.raw)
							time.sleep(0.1)
							header_data = serial.read(2)
							if (len(header_data) < 2): continue
							receive_header.load(bytes_to_int(header_data))
							if (receive_header.get_op_code() == OP_CODE.ACK): break"""
	print("init success")

	sent = [header()] * 16  # make sent box <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	instruction = motor_instruction();
	state = MCU_state();
	while True:	# mainloop
		print("\n",
			f"MOTOR_INFO: {OP_CODE.MOTOR_CONFIG}",
			f"MOTOR_POS: {OP_CODE.MOTOR_POS}",
			f"MOTOR_MOVE: {OP_CODE.MOTOR_MOVE}",
			sep="\n"
		)
		resp = input(": ")
		try: resp = int(resp);
		except: print("INPUT ERROR\n"); continue

		# receiving + args
		if (resp == OP_CODE.MOTOR_CONFIG):
			send_header.set(OP_CODE.MOTOR_CONFIG, 0x1, 0x0)
			serial.write(send_header.raw)

			continue;

		elif (resp == OP_CODE.MOTOR_POS):
			send_header.set(OP_CODE.MOTOR_POS, 0x1, 0x0)
			instruction.set(0, 0);

			serial.write(send_header.raw)
			serial.write(instruction.raw)

			state.load(serial.read(18))
			print(state)

		elif (resp == OP_CODE.MOTOR_MOVE):
			resp = input("NUMBER OF STEPS: ")
			# TODO: GET PULSE DELAY <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			try: instruction.set(int(resp), 0)
			except: print("INPUT ERROR\n"); continue
			send_header.set(OP_CODE.MOTOR_MOVE, 0x1, 0x0)

			serial.write(send_header.raw)
			serial.write(instruction.raw)

			state.load(serial.read(18))
			print(state)


# reset_input_buffer()
# reset_output_buffer()


# startup
if __name__ == "__main__":
	if "-help" in sys.argv:
		print(
			"(*)\t\t\t-> required",
			"(,)\t\t\t-> requires value",
			"[-adapter](*,)\t\t-> adapter/connection name",
			"[-set-baud-rate](,)\t-> set the baud rate of the connection",
			"[-set-timeout](,)\t-> set the time out (seconds)",
		sep="\n", end="\n\n")
	else:
		args = {}
		
		if "-adapter" in sys.argv:			args["adapter"] = sys.argv[sys.argv.index("-adapter") + 1]
		if "-set-baud-rate" in sys.argv:	args["baudRate"] = int(sys.argv[sys.argv.index("-set-baud-rate") + 1])
		if "-set-timeout" in sys.argv:		args["timeout"] = float(sys.argv[sys.argv.index("-set-timeout") + 1])

		main(**args)



# TODO: add crc support in arduino and in send code