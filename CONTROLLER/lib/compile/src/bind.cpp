#include "pybind11/pybind11.h"
namespace py = pybind11;

#include "int.hpp"
#include "com.hpp"



PYBIND11_MODULE(py_lib, handle) {
	handle.doc() = "communication module for python using C++";

	handle.attr("SYNC_BYTE") = py::int_(SYNC_BYTE);

	handle.def("new_instruction", [](
		uint16_t id,
		uint8_t action,
		double target,
		double max_vel,
		double max_acc,
		uint8_t micro_step,
		uint8_t srd_mode
	) {
		uint8_t* data = new_instruction(id, action, target, max_vel, max_acc, micro_step, srd_mode);
		py::bytes result = py::reinterpret_steal<py::object>(PYBIND11_BYTES_FROM_STRING_AND_SIZE((char*)data, 28));
		delete[] data;
		return result;
	});
	handle.def("get_instruction_data", [](char* handshake) {
		double		target;
		double		max_vel;
		double		max_acc;
		uint8_t		micro_step;
		uint8_t		srd_mode;
		uint8_t		action;
		uint8_t		id;
		uint16_t	crc;
		get_instruction_data((uint8_t*)handshake, &target, &max_vel, &max_acc, &micro_step, &srd_mode, &action, &id, &crc);
		return std::make_tuple(target, max_vel, max_acc, micro_step, srd_mode, action, id, crc);
	});
	// overload handshake functions so that bytes like object can be passed
	handle.def("new_handshake", [](uint8_t motor_count, uint8_t init_0, uint32_t baud) {
		uint8_t* data = new_handshake(motor_count, init_0, baud);
		py::bytes result = py::reinterpret_steal<py::object>(PYBIND11_BYTES_FROM_STRING_AND_SIZE((char*)data, 6));
		delete[] data;
		return result;
	});
	handle.def("get_handshake_data", [](char* handshake) {
		uint8_t		motor_count;
		uint8_t		init_0;
		uint32_t	baud;
		uint16_t	crc;
		get_handshake_data(*((uint32_t*)handshake), &motor_count, &init_0, &baud, &crc);
		return std::make_tuple(motor_count, init_0, baud, crc);
	});

	py::enum_<flags> flags_py(handle, "flags", py::arithmetic());
	flags_py.attr("EXEC") =		flags::EXEC;
	flags_py.attr("OVERRIDE") =	flags::OVERRIDE;
	flags_py.attr("SYNC") =		flags::SYNC;
	flags_py.attr("POLL") =		flags::POLL;
	flags_py.export_values();
}