#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "../../../include/Livelybot_Driver.h"
#include <iostream>
#include <stdint.h>

namespace py = pybind11;


PYBIND11_MODULE(Livelybot_Driver_bind,m)
{   
    py::class_<motor_fb_space_s>(m, "MotorFbSpaceS")
        .def(py::init<>())
        .def_readwrite("motor_id", &motor_fb_space_s::motor_id)
        .def_readwrite("motor_cmd", &motor_fb_space_s::motor_cmd)
        .def_readwrite("position", &motor_fb_space_s::position)
        .def_readwrite("velocity", &motor_fb_space_s::velocity)
        .def_readwrite("torque", &motor_fb_space_s::torque);

    py::class_<imu_space_s>(m, "ImuSpaceS")
        .def(py::init<>())
        .def_readwrite("accX", &imu_space_s::accX)
        .def_readwrite("accY", &imu_space_s::accY)
        .def_readwrite("accZ", &imu_space_s::accZ)
        .def_readwrite("magX", &imu_space_s::magX)
        .def_readwrite("magY", &imu_space_s::magY)
        .def_readwrite("magZ", &imu_space_s::magZ)
        .def_readwrite("angVelX", &imu_space_s::angVelX)
        .def_readwrite("angVelY", &imu_space_s::angVelY)
        .def_readwrite("angVelZ", &imu_space_s::angVelZ)
        .def_readwrite("angle_yaw", &imu_space_s::angle_yaw)
        .def_readwrite("angle_roll", &imu_space_s::angle_roll)
        .def_readwrite("angle_pitch", &imu_space_s::angle_pitch);

    py::class_<motor_fb_s>(m, "MotorFbS")
        .def(py::init<>())
        .def_readwrite("motor", &motor_fb_s::motor)
        .def_property("data", [](motor_fb_s& self) {
            py::array_t<uint8_t> arr(MOTOR_STATUS_LENGTH);
            auto buf = arr.request();
            uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
            for (size_t i = 0; i < MOTOR_STATUS_LENGTH; ++i) {
                ptr[i] = self.data[i];
            }
            return arr;
        }, [](motor_fb_s& self, py::array_t<uint8_t> arr) {
            auto buf = arr.request();
            uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
            for (size_t i = 0; i < MOTOR_STATUS_LENGTH; ++i) {
                self.data[i] = ptr[i];
            }
        });
    
    py::class_<imu_s>(m, "ImuS")
        .def(py::init<>())
        .def_readwrite("imu_data", &imu_s::imu_data)
        .def_property("data", [](imu_s& self) {
            py::array_t<uint8_t> arr(YJ901S_DATA_SIZE);
            auto buf = arr.request();
            uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
            for (size_t i = 0; i < YJ901S_DATA_SIZE; ++i) {
                ptr[i] = self.data[i];
            }
            return arr;
        }, [](imu_s& self, py::array_t<uint8_t> arr) {
            auto buf = arr.request();
            uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
            for (size_t i = 0; i < YJ901S_DATA_SIZE; ++i) {
                self.data[i] = ptr[i];
            }
    });

    py::class_<motor_set_space_s>(m, "MotorSetSpaceS")
        .def(py::init<>())
        .def_readwrite("motor_id", &motor_set_space_s::motor_id)
        .def_readwrite("motor_cmd", &motor_set_space_s::motor_cmd)
        .def_readwrite("position", &motor_set_space_s::position)
        .def_readwrite("velocity", &motor_set_space_s::velocity)
        .def_readwrite("torque", &motor_set_space_s::torque)
        .def_readwrite("kp", &motor_set_space_s::kp)
        .def_readwrite("kd", &motor_set_space_s::kd);

    py::class_<motor_set_s>(m, "MotorSetS")
        .def(py::init<>())
        .def_readwrite("motor", &motor_set_s::motor)
        .def_property("data", [](motor_set_s& self) {
            py::array_t<uint8_t> arr(MOTOR_SET_LENGTH);
            auto buf = arr.request();
            uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
            for (size_t i = 0; i < MOTOR_SET_LENGTH; ++i) {
                ptr[i] = self.data[i];
            }
            return arr;
        }, [](motor_set_s& self, py::array_t<uint8_t> arr) {
            auto buf = arr.request();
            uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
            for (size_t i = 0; i < MOTOR_SET_LENGTH; ++i) {
                self.data[i] = ptr[i];
            }
        });

    py::class_<Motor_Status>(m, "Motor_Status")
        .def(py::init<>())
        .def_property("motor_fb1", [](Motor_Status& self) {
            py::array_t<uint8_t> arr(14 * sizeof(motor_fb_s));
            auto buf = arr.request();
            uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
            memcpy(ptr, self.motor_fb1, 14 * sizeof(motor_fb_s));
            return arr;
        }, [](Motor_Status& self, py::array_t<uint8_t> arr) {
            auto buf = arr.request();
            uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
            memcpy(self.motor_fb1, ptr, 14 * sizeof(motor_fb_s));
        })
        .def_property("motor_fb2", [](Motor_Status& self) {
            py::array_t<uint8_t> arr(14 * sizeof(motor_fb_s));
            auto buf = arr.request();
            uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
            memcpy(ptr, self.motor_fb2, 14 * sizeof(motor_fb_s));
            return arr;
        }, [](Motor_Status& self, py::array_t<uint8_t> arr) {
            auto buf = arr.request();
            uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
            memcpy(self.motor_fb2, ptr, 14 * sizeof(motor_fb_s));
        })
        .def_readwrite("myimu", &Motor_Status::myimu)
        .def_property("foot_sensor1", [](Motor_Status& self) {
            py::array_t<uint8_t> arr(3);
            auto buf = arr.request();
            uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
            for (size_t i = 0; i < 3; ++i) {
                ptr[i] = self.foot_sensor1[i];
            }
            return arr;
        }, [](Motor_Status& self, py::array_t<uint8_t> arr) {
            auto buf = arr.request();
            uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
            for (size_t i = 0; i < 3; ++i) {
                self.foot_sensor1[i] = ptr[i];
            }
        })
        .def_property("foot_sensor2", [](Motor_Status& self) {
            py::array_t<uint8_t> arr(3);
            auto buf = arr.request();
            uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
            for (size_t i = 0; i < 3; ++i) {
                ptr[i] = self.foot_sensor2[i];
            }
            return arr;
        }, [](Motor_Status& self, py::array_t<uint8_t> arr) {
            auto buf = arr.request();
            uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
            for (size_t i = 0; i < 3; ++i) {
                self.foot_sensor2[i] = ptr[i];
            }
        });

    py::enum_<tranfer_send_type_e>(m, "TranferSendTypeE")
        .value("ANG2POS", tranfer_send_type_e::ANG2POS)
        .value("RAD2POS", tranfer_send_type_e::RAD2POS)
        .value("TOR2TOR", tranfer_send_type_e::TOR2TOR)
        .export_values();

    py::enum_<tranfer_rec_type_e>(m, "TranferRecTypeE")
        .value("POS2ANG", tranfer_rec_type_e::POS2ANG)
        .value("POS2RAD", tranfer_rec_type_e::POS2RAD)
        .export_values();
    
    py::class_<Livelybot_Driver>(m,"Livelybot_Driver")
        .def(py::init<const std::string&>())
        .def("get_motor_state",&Livelybot_Driver::get_motor_state)
        .def("get_imu_data",&Livelybot_Driver::get_imu_data)
        .def("get_footsensor_data",[](Livelybot_Driver &self, uint8_t switch_can){
            uint8_t* array = self.get_footsensor_data(switch_can);
            
            // 创建一个 numpy 数组来包装指针数组
            py::array_t<uint8_t> result({3}, {sizeof(uint8_t)}, array);
            return result;
        })
        .def("spi_send",&Livelybot_Driver::spi_send)
        .def("set_motor_position", py::overload_cast<int8_t, int32_t>(&Livelybot_Driver::set_motor_position))
        .def("set_motor_position", py::overload_cast<int8_t, int32_t, int32_t, int32_t, float, float>(&Livelybot_Driver::set_motor_position))
        .def("set_motor_velocity",&Livelybot_Driver::set_motor_velocity)
        .def("set_motor_torque",&Livelybot_Driver::set_motor_torque)
        .def("transfer_send",&Livelybot_Driver::transfer_send)
        .def("transfer_rec",&Livelybot_Driver::transfer_rec);
}