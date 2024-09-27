/*MIT License

Copyright (c) 2024 Kenneth Martinez

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
#include <atomic>
#include <chrono>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "grasp_rmd_ros2/rmd_hardware_interface.hpp"

#if __has_include(<pthread.h>) && __has_include(<sched.h>)
  #include <pthread.h>
  #include <sched.h>
  #define GRASP_RMD_ROS2__THREAD_PRIORITY
#endif

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <myactuator_rmd/actuator_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "myactuator_rmd_hardware/conversions.hpp"

namespace rmd_hardware_interface {

using CallbackReturn = GRASPActuatorsHardwareInterface::CallbackReturn;

GRASPActuatorsHardwareInterface::~GRASPActuatorsHardwareInterface() {
    on_cleanup(rclcpp_lifecycle::State());
}

CallbackReturn GRASPActuatorsHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // determine dynamically the number of actuators by info in URDF or configuration file
    size_t num_joints = info.joints.size();
    
    actuators_.resize(num_joints);
    actuator_ids_.resize(num_joints);
    position_states_.resize(num_joints);
    velocity_states_.resize(num_joints);
    effort_states_.resize(num_joints);
    position_commands_.resize(num_joints);
    velocity_commands_.resize(num_joints);
    effort_commands_.resize(num_joints);
    position_interfaces_running_.resize(num_joints, false);
    velocity_interfaces_running_.resize(num_joints, false);
    effort_interfaces_running_.resize(num_joints, false);

    return CallbackReturn::SUCCESS;
}

CallbackReturn GRASPActuatorsHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    // Parse parameters and initialize actuators
    for (std::size_t i = 0; i < actuators_.size(); ++i) {
        ifname_ = info_.hardware_parameters["ifname"];
        actuator_ids_[i] = std::stoi(info_.hardware_parameters["actuator_id_" + std::to_string(i)]);
        driver_ = std::make_unique<myactuator_rmd::CanDriver>(ifname_);
        actuators_[i] = std::make_unique<myactuator_rmd::ActuatorInterface>(*driver_, actuator_ids_[i]);

        if (!actuators_[i]) {
            RCLCPP_FATAL(getLogger(), "Failed to create actuator interface for actuator %zu!", i);
            return CallbackReturn::ERROR;
        }
    }

    stop_async_thread_.store(false);
    if (!startAsyncThread(cycle_time_)) {
        RCLCPP_FATAL(getLogger(), "Failed to start async thread!");
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn GRASPActuatorsHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(getLogger(), "Actuators successfully activated!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn GRASPActuatorsHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    stopAsyncThread();
    RCLCPP_INFO(getLogger(), "Actuators successfully deactivated!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn GRASPActuatorsHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
    stopAsyncThread();
    for (auto & actuator : actuators_) {
        actuator->shutdownMotor();
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn GRASPActuatorsHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) {
    stopAsyncThread();
    for (auto & actuator : actuators_) {
        actuator->shutdownMotor();
    }
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GRASPActuatorsHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < actuators_.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &effort_states_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GRASPActuatorsHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < actuators_.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &effort_commands_[i]));
    }
    return command_interfaces;
}

hardware_interface::return_type GRASPActuatorsHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // Read all actuators' states
    for (std::size_t i = 0; i < actuators_.size(); ++i) {
        auto feedback = actuators_[i]->getMotorStatus2();
        position_states_[i] = degToRad(feedback.shaft_angle);
        velocity_states_[i] = degToRad(feedback.shaft_speed);
        effort_states_[i] = currentToTorque(feedback.current, torque_constant_);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type GRASPActuatorsHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // Write commands to all actuators
    for (std::size_t i = 0; i < actuators_.size(); ++i) {
        if (position_interfaces_running_[i]) {
            actuators_[i]->sendPositionAbsoluteSetpoint(radToDeg(position_commands_[i]), max_velocity_);
        } else if (velocity_interfaces_running_[i]) {
            actuators_[i]->sendVelocitySetpoint(radToDeg(velocity_commands_[i]));
        } else if (effort_interfaces_running_[i]) {
            actuators_[i]->sendTorqueSetpoint(effort_commands_[i], torque_constant_);
        }
    }
    return hardware_interface::return_type::OK;
}

void GRASPActuatorsHardwareInterface::asyncThread(const std::chrono::milliseconds & cycle_time) {
    while (!stop_async_thread_) {
        auto start = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < actuators_.size(); ++i) {
            if (position_interfaces_running_[i]) {
                actuators_[i]->sendPositionAbsoluteSetpoint(radToDeg(position_commands_[i]), max_velocity_);
            } else if (velocity_interfaces_running_[i]) {
                actuators_[i]->sendVelocitySetpoint(radToDeg(velocity_commands_[i]));
            } else if (effort_interfaces_running_[i]) {
                actuators_[i]->sendTorqueSetpoint(effort_commands_[i], torque_constant_);
            }
        }
        std::this_thread::sleep_until(start + cycle_time);
    }
}

bool GRASPActuatorsHardwareInterface::startAsyncThread(const std::chrono::milliseconds & cycle_time) {
    if (!async_thread_.joinable()) {
        async_thread_ = std::thread(&GRASPActuatorsHardwareInterface::asyncThread, this, cycle_time);
        return true;
    }
    RCLCPP_WARN(getLogger(), "Command thread already running!");
    return false;
}

void GRASPActuatorsHardwareInterface::stopAsyncThread() {
    if (async_thread_.joinable()) {
        stop_async_thread_.store(true);
        async_thread_.join();
    }
}

rclcpp::Logger GRASPActuatorsHardwareInterface::getLogger() {
    return rclcpp::get_logger("GRASPActuatorsHardwareInterface");
}

}  // namespace rmd_hardware_interface

PLUGINLIB_EXPORT_CLASS(rmd_hardware_interface::GRASPActuatorsHardwareInterface, hardware_interface::SystemInterface)