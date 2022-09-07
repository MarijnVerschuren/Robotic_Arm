# This file was automatically generated by SWIG (http://www.swig.org).
# Version 4.0.2
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

from sys import version_info as _swig_python_version_info
if _swig_python_version_info < (2, 7, 0):
    raise RuntimeError("Python 2.7 or later required")

# Import the low-level C/C++ module
if __package__ or "." in __name__:
    from . import _py_lib
else:
    import _py_lib

try:
    import builtins as __builtin__
except ImportError:
    import __builtin__

def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except __builtin__.Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)


def _swig_setattr_nondynamic_instance_variable(set):
    def set_instance_attr(self, name, value):
        if name == "thisown":
            self.this.own(value)
        elif name == "this":
            set(self, name, value)
        elif hasattr(self, name) and isinstance(getattr(type(self), name), property):
            set(self, name, value)
        else:
            raise AttributeError("You cannot add instance attributes to %s" % self)
    return set_instance_attr


def _swig_setattr_nondynamic_class_variable(set):
    def set_class_attr(cls, name, value):
        if hasattr(cls, name) and not isinstance(getattr(cls, name), property):
            set(cls, name, value)
        else:
            raise AttributeError("You cannot add class attributes to %s" % cls)
    return set_class_attr


def _swig_add_metaclass(metaclass):
    """Class decorator for adding a metaclass to a SWIG wrapped class - a slimmed down version of six.add_metaclass"""
    def wrapper(cls):
        return metaclass(cls.__name__, cls.__bases__, cls.__dict__.copy())
    return wrapper


class _SwigNonDynamicMeta(type):
    """Meta class to enforce nondynamic attributes (no new attributes) for a class"""
    __setattr__ = _swig_setattr_nondynamic_class_variable(type.__setattr__)


class OP_CODE(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    MOTOR_CONFIG = _py_lib.OP_CODE_MOTOR_CONFIG
    MOTOR_MOVE = _py_lib.OP_CODE_MOTOR_MOVE
    MOTOR_POS = _py_lib.OP_CODE_MOTOR_POS
    ACK = _py_lib.OP_CODE_ACK
    NACK = _py_lib.OP_CODE_NACK
    INIT = _py_lib.OP_CODE_INIT

    def __init__(self):
        _py_lib.OP_CODE_swiginit(self, _py_lib.new_OP_CODE())
    __swig_destroy__ = _py_lib.delete_OP_CODE

# Register OP_CODE in _py_lib:
_py_lib.OP_CODE_swigregister(OP_CODE)

class CRC_TYPE(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    CRC_4 = _py_lib.CRC_TYPE_CRC_4
    CRC_8 = _py_lib.CRC_TYPE_CRC_8
    CRC_16 = _py_lib.CRC_TYPE_CRC_16
    CRC_32 = _py_lib.CRC_TYPE_CRC_32
    CRC_64 = _py_lib.CRC_TYPE_CRC_64

    def __init__(self):
        _py_lib.CRC_TYPE_swiginit(self, _py_lib.new_CRC_TYPE())
    __swig_destroy__ = _py_lib.delete_CRC_TYPE

# Register CRC_TYPE in _py_lib:
_py_lib.CRC_TYPE_swigregister(CRC_TYPE)


def crc_64(buffer, bytes):
    return _py_lib.crc_64(buffer, bytes)

def crc_32(buffer, bytes):
    return _py_lib.crc_32(buffer, bytes)

def crc_16(buffer, bytes):
    return _py_lib.crc_16(buffer, bytes)

def crc_8(buffer, bytes):
    return _py_lib.crc_8(buffer, bytes)

def crc_4(buffer, bytes):
    return _py_lib.crc_4(buffer, bytes)

def error_correct(buffer, bytes, crc, type, bit_cutoff=0):
    return _py_lib.error_correct(buffer, bytes, crc, type, bit_cutoff)
class header(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    _raw = property(_py_lib.header__raw_get, _py_lib.header__raw_set)

    def __init__(self):
        _py_lib.header_swiginit(self, _py_lib.new_header())

    def load(self, value):
        return _py_lib.header_load(self, value)

    def set(self, op_code, msg_id, motor_id, poly_data=1):
        return _py_lib.header_set(self, op_code, msg_id, motor_id, poly_data)

    def get_op_code(self):
        return _py_lib.header_get_op_code(self)

    def get_msg_id(self):
        return _py_lib.header_get_msg_id(self)

    def get_motor_id(self):
        return _py_lib.header_get_motor_id(self)

    def get_poly_data(self):
        return _py_lib.header_get_poly_data(self)

    def get_crc(self):
        return _py_lib.header_get_crc(self)
    __swig_destroy__ = _py_lib.delete_header

# Register header in _py_lib:
_py_lib.header_swigregister(header)

class motor_instruction(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    steps = property(_py_lib.motor_instruction_steps_get, _py_lib.motor_instruction_steps_set)
    pulse_delay = property(_py_lib.motor_instruction_pulse_delay_get, _py_lib.motor_instruction_pulse_delay_set)
    crc = property(_py_lib.motor_instruction_crc_get, _py_lib.motor_instruction_crc_set)

    def set(self, steps, pulse_delay):
        return _py_lib.motor_instruction_set(self, steps, pulse_delay)

    def __init__(self):
        _py_lib.motor_instruction_swiginit(self, _py_lib.new_motor_instruction())
    __swig_destroy__ = _py_lib.delete_motor_instruction

# Register motor_instruction in _py_lib:
_py_lib.motor_instruction_swigregister(motor_instruction)

class MCU_state(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    pos = property(_py_lib.MCU_state_pos_get, _py_lib.MCU_state_pos_set)
    job = property(_py_lib.MCU_state_job_get, _py_lib.MCU_state_job_set)
    crc = property(_py_lib.MCU_state_crc_get, _py_lib.MCU_state_crc_set)

    def load(self, buffer):
        return _py_lib.MCU_state_load(self, buffer)

    def __init__(self):
        _py_lib.MCU_state_swiginit(self, _py_lib.new_MCU_state())
    __swig_destroy__ = _py_lib.delete_MCU_state

# Register MCU_state in _py_lib:
_py_lib.MCU_state_swigregister(MCU_state)


