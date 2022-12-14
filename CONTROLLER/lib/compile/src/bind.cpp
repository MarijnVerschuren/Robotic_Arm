#include "pybind11/pybind11.h"
namespace py = pybind11;

#include "int.hpp"
#include "com.hpp"



PYBIND11_MODULE(py_lib, handle) {
	handle.doc() = "communication module for python using C++";

	handle.attr("SYNC_BYTE") = py::int_(SYNC_BYTE);

	handle.def("new_MCU_State", [](
		double vel,
		double acc,
		int32_t pos_rotation,
		int32_t target_rotation,
		uint16_t pos_angle,
		uint16_t target_angle,
		uint16_t raw_angle,
		uint16_t instrution_id,
		uint8_t micro_step,
		uint8_t srd_mode,
		uint8_t id,
		uint8_t queue_size,
		uint8_t queue_index
	) {
		uint8_t* data = new_MCU_State(vel, acc, pos_rotation, target_rotation, pos_angle, target_angle, raw_angle, instrution_id, micro_step, srd_mode, id, queue_size, queue_index);
		py::bytes result = py::reinterpret_steal<py::object>(PYBIND11_BYTES_FROM_STRING_AND_SIZE((char*)data, 32));
		delete[] data;
		return result;
	});
	handle.def("get_MCU_State_data", [](char* state) {
		double vel;
		double acc;
		int32_t pos_rotation;
		int32_t target_rotation;
		uint16_t pos_angle;
		uint16_t target_angle;
		uint16_t raw_angle;
		uint16_t instrution_id;
		uint8_t micro_step;
		uint8_t srd_mode;
		uint8_t id;
		uint8_t queue_size;
		uint8_t queue_index;
		get_MCU_State_data((uint8_t*)state, &vel, &acc, &pos_rotation, &target_rotation, &pos_angle, &target_angle, &raw_angle, &instrution_id, &micro_step, &srd_mode, &id, &queue_size, &queue_index);
		return std::make_tuple(vel, acc, pos_rotation, target_rotation, pos_angle, target_angle, raw_angle, instrution_id, micro_step, srd_mode, id, queue_size, queue_index);
	});

	handle.def("new_MCU_Instruction", [](
		uint16_t id,
		uint8_t action,
		double target,
		double max_vel,
		double max_acc,
		uint8_t micro_step,
		uint8_t srd_mode,
		uint8_t dir
	) {
		uint16_t instrution_id;
		uint8_t* data = new_MCU_Instruction(id, action, target, max_vel, max_acc, micro_step, srd_mode, dir, &instrution_id);
		py::bytes result = py::reinterpret_steal<py::object>(PYBIND11_BYTES_FROM_STRING_AND_SIZE((char*)data, 32));
		delete[] data;
		return std::make_tuple(result, instrution_id);
	});
	handle.def("get_MCU_Instruction_data", [](char* instrution) {
		double		target;
		double		max_vel;
		double		max_acc;
		uint8_t		micro_step;
		uint8_t		srd_mode;
		uint8_t		action;
		uint8_t		dir;
		uint8_t		id;
		uint16_t	instrution_id;
		uint16_t	crc;
		get_MCU_Instruction_data((uint8_t*)instrution, &target, &max_vel, &max_acc, &micro_step, &srd_mode, &action, &dir, &id, &instrution_id, &crc);
		return std::make_tuple(target, max_vel, max_acc, micro_step, srd_mode, action, dir, id, instrution_id, crc);
	});

	// overload handshake functions so that bytes like object can be passed
	handle.def("new_CTRL_Handshake", [](uint8_t motor_count, uint8_t init_0, uint32_t baud) {
		uint8_t* data = new_CTRL_Handshake(motor_count, init_0, baud);
		py::bytes result = py::reinterpret_steal<py::object>(PYBIND11_BYTES_FROM_STRING_AND_SIZE((char*)data, 6));
		delete[] data;
		return result;
	});
	handle.def("get_CTRL_Handshake_data", [](char* handshake) {
		uint8_t		motor_count;
		uint8_t		init_0;
		uint32_t	baud;
		uint16_t	crc;
		get_CTRL_Handshake_data(*((uint32_t*)handshake), &motor_count, &init_0, &baud, &crc);
		return std::make_tuple(motor_count, init_0, baud, crc);
	});

	handle.def("crc16_dnp", [](std::string& data) {
		return crc16_dnp(data.c_str(), data.length());
	});

	py::enum_<ACTION_FLAGS> action_flags_py(handle, "ACTION_FLAGS", py::arithmetic());
	action_flags_py.attr("EXEC") =			ACTION_FLAGS::EXEC;
	action_flags_py.attr("OVERRIDE") =		ACTION_FLAGS::OVERRIDE;
	action_flags_py.attr("SYNC") =			ACTION_FLAGS::SYNC;
	action_flags_py.attr("POLL") =			ACTION_FLAGS::POLL;
	action_flags_py.export_values();

	py::enum_<RETURN_FLAGS> return_flags_py(handle, "RETURN_FLAGS", py::arithmetic());
	return_flags_py.attr("OK") =			RETURN_FLAGS::OK;
	return_flags_py.attr("CRC_FIXED") =		RETURN_FLAGS::CRC_FIXED;
	return_flags_py.attr("CRC_ERROR") =		RETURN_FLAGS::CRC_ERROR;
	return_flags_py.attr("ERROR_FIXED") =	RETURN_FLAGS::ERROR_FIXED;
	return_flags_py.attr("ERROR") =			RETURN_FLAGS::ERROR;
	return_flags_py.export_values();
}
