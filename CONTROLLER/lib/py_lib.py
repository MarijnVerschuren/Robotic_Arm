import struct

from . import py_lib_wrap



class OP_CODE(py_lib_wrap.OP_CODE):
	"""opcode enum"""

class CRC_TYPE(py_lib_wrap.OP_CODE):
	"""opcode enum"""

class header(py_lib_wrap.header):
	def __init__(self) -> None:
		super(header, self).__init__()

	@property
	def raw(self) -> bytes: return struct.pack("<L", self._raw)

	def __str__(self) -> str:
		return f"{{op_code: {self.get_op_code()}, msg_id: {self.get_msg_id()}, motor_id: {self.get_motor_id()}, poly_data: {self.get_poly_data()}, crc: {self.get_crc()}}}"

	def __repr__(self) -> str: return str(self)


class message_data:
	data_len = 0

	@property  # all data classes have to have the raw property
	def raw(): return None
	"""class inherited by all message data classes (py only) (c/c++ uses void*)"""

class motor_instruction(py_lib_wrap.motor_instruction, message_data):
	data_len = 16
	def __init__(self) -> None:
		super(motor_instruction, self).__init__();

	@property
	def raw(self) -> bytes: return struct.pack("<qLL", self.steps, self.pulse_delay, self.crc)

	def __str__(self) -> str:
		return f"{{steps: {self.steps}, pulse_delay: {self.pulse_delay}, crc: {self.crc}}}"

	def __repr__(self) -> str: return str(self)



class MCU_state(py_lib_wrap.MCU_state, message_data):
	data_len = 18
	def __init__(self) -> None:
		super(MCU_state, self).__init__();

	@property
	def raw(self) -> bytes: return struct.pack("<qqH", self.pos, self.job, self.crc)

	def __str__(self) -> str:
		return f"{{pos: {self.pos}, job: {self.job}, crc: {self.crc}}}"

	def __repr__(self) -> str: return str(self)




def bytes_to_int(data: bytes) -> int:
	return int.from_bytes(data, "little")


def crc_64(data: bytes, bytes: int, pack: bool = True) -> int or bytes:
	crc = py_lib_wrap.crc_64(data, bytes)
	return struct.pack("<Q", crc) if pack else crc

def crc_32(data: bytes, bytes: int, pack: bool = True) -> int or bytes:
	crc = py_lib_wrap.crc_32(data, bytes)
	return struct.pack("<L", crc) if pack else crc

def crc_16(data: bytes, bytes: int, pack: bool = True) -> int or bytes:
	crc = py_lib_wrap.crc_16(data, bytes)
	return struct.pack("<H", crc) if pack else crc

def crc_8(data: bytes, bytes: int, pack: bool = True) -> int or bytes:
	crc = py_lib_wrap.crc_8(data, bytes)
	return struct.pack("<B", crc) if pack else crc

def crc_4(data: bytes, bytes: int, pack: bool = True) -> int or bytes:
	crc = py_lib_wrap.crc_4(data, bytes)
	return struct.pack("<B", crc) if pack else crc

def error_correct(buffer: bytes, bytes: int, crc: int, type: CRC_TYPE, bit_cutoff: int = 0) -> bool:
	return py_lib_wrap.error_correct(buffer, bytes, crc, type, bit_cutoff)
