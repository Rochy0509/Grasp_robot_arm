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

#ifndef RMD_HARDWARE_INTERFACE_HPP_
#define RMD_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <cstdint>
#include <thread>

//myactuator_rmd sdk files
#include <myactuator_rmd/driver/can_driver.hpp>
#include <myactuator_rmd/actuator_interface.hpp>

//ros2_control section
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace rmd_hardware_interface{

    /**\class GRASPActuatorsHardwareInterface
   * \brief
   *    Hardware interface for the MyActuator RMD X-series actuators based on CAN
  */

class GRASPActuatorsHardwareInterface : public hardware_interface::SystemInterface{
    public:
      RCLCPP_SHARED_PTR_DEFINITIONS(GRASPActuatorsHardwareInterface);

      GRASPActuatorsHardwareInterface() = default;
      GRASPActuatorsHardwareInterface(GRASPActuatorsHardwareInterface const&) = default;
      GRASPActuatorsHardwareInterface& operator = (GRASPActuatorsHardwareInterface const&) = default;
      GRASPActuatorsHardwareInterface(GRASPActuatorsHardwareInterface&&) = default;
      GRASPActuatorsHardwareInterface& operator = (GRASPActuatorsHardwareInterface&&) = default;

      /**\fn ~GRASPActuatorsHardwareInterface
       * \brief
       *    Class destructor
       *    check shutdown went cleanly
      */
      ~GRASPActuatorsHardwareInterface();

      /**\fn on_init
       *  \brief
       *    Callback function for init transition
       *    Initialize all members variables and process the parameters from the info argument
       * 
       *    \param [in] info
       *        Hardware info contained in URDF file
       *    \return
       *        Value for sucess or failure from callback
      */    
      hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

      /**\fn on_configure
       *  \brief
       *    Callback function for setup transition
       *    Sets up hardware and everything 
       * 
       *    \param [in] previous state
       *        Last state
       *    \return
       *        Value for sucess or failure from callback
      */
      hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

      /**\fn export_state_interface
       *  \brief
       *    Export state interfaces created by actuators exposion.
       * 
       *   \return
       *        state interface from actuators
       *     
      */  
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      /**\fn export_command_interface
       *  \brief
       *    Export command interfaces created by actuators exposion.
       * 
       *   \return
       *        command interface from actuators
       *     
      */
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      /**\fn on_activate
       *  \brief
       *    Callback function for activation transition
       *    Activates once hardware is power or enabled
       * 
       *   \param state
       *        Previous State
       *   \return
       *        Value success or failure from callback
       *     
      */
      hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

      /**\fn on_deactivete
       *  \brief
       *    Callback function for deactivation transition
       *    Deactivate hardware
       *    
       *   \param state
       *        Previous state    
       *   \return
       *        Value for succes or failure from callback
       *     
      */
      hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

      /**\fn read
       *  \brief
       *    Read current state of actuators
       * 
       *   \param time
       *        current time
       *   \param period
       *        time since last read
       * 
       *   \return
       *        Value for succes or failure from callback
       *     
      */
      hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

      /**\fn write
       *  \brief
       *    Write next command to actuators
       * 
       *   \param time
       *        current time       
       *   \param period
       *        time since last write
       * 
       *   \return
       *        Value for succes or failure from callback
       *     
      */
      hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
      
      /**\fn on_error
       * \brief
       *    Callback function for error transition
       *    Resets actuator
       * 
       * \param[in] previous_state
       *    Previous state that we are transitioning from
       * \return
       *    Value indicating success, error or failure of the callback
      */
      hardware_interface::CallbackReturn on_error(rclcpp_lifecycle::State const& previous_state) override;

      /**\fn on_cleanup
       * \brief
       *    Callback function for cleanup transition
       *    Gracefully shuts down the actuator
       * 
       * \param[in] previous_state
       *    Previous state that we are transitioning from
       * \return
       *    Value indicating success, error or failure of the callback
      */
      hardware_interface::CallbackReturn on_cleanup(rclcpp_lifecycle::State const& previous_state) override;
      
      /**\fn on_shutdown
       * \brief
       *    Callback function for shutdown transition
       *    Gracefully shuts down the actuator
       * 
       * \param[in] previous_state
       *    Previous state that we are transitioning from
       * \return
       *    Value indicating success, error or failure of the callback
      */
      hardware_interface::CallbackReturn on_shutdown(rclcpp_lifecycle::State const& previous_state) override;
    
    protected:      
      /**\fn getLogger
       * \brief
       *    Get the logger used for console output
       * 
       * \return
       *    The logger
      */
      static rclcpp::Logger getLogger();

      /**\fn asyncThread
       * \brief
       *    The asynchronous command thread used to communicate with the hardware
       *    that performs a combined read and write
       * 
       * \param[in] cycle_time
       *    The cycle time that the asynchronous thread should run at
      */
      void asyncThread(std::chrono::milliseconds const& cycle_time);

      /**\fn startAsyncThread
       * \brief
       *    Start the asynchronous command thread used to communicate with the hardware
       * 
       * \param[in] cycle_time
       *    The cycle time that the asynchronous thread should run at
       * \return
       *    Boolean variable indicating successful start of the async thread or failure
      */
      [[nodiscard]]
      bool startAsyncThread(std::chrono::milliseconds const& cycle_time);
      
      /**\fn stopAsyncThread
       * \brief
       *    Stop the asynchronous command thread used to communicate with the hardware
      */
      void stopAsyncThread();

      std::string ifname_;
      std::uint32_t actuator_id_;
      double torque_constant_;
      double max_velocity_;
      std::chrono::milliseconds timeout_;

      // Buffers only used by the main thread
      double position_state_;
      double velocity_state_;
      double effort_state_;
      double position_command_;
      double velocity_command_;
      double effort_command_;
      // std::unique_ptr<LowPassFilter> velocity_low_pass_filter_;
      // std::unique_ptr<LowPassFilter> effort_low_pass_filter_;
      std::unique_ptr<ButterworthFilter> velocity_butter_worth_filter_;
      std::unique_ptr<ButterworthFilter> effort_butter_worth_filter_;

      // The command thread reads and writes from the actuator cyclically
      std::thread async_thread_;
      std::chrono::milliseconds cycle_time_;
      // Never accessed by both threads at the same time
      std::unique_ptr<myactuator_rmd::CanDriver> driver_;
      std::unique_ptr<myactuator_rmd::ActuatorInterface> actuator_interface_;
      myactuator_rmd::Feedback feedback_;
      // Shared between the two threads
      std::atomic<bool> stop_async_thread_;
      
      std::atomic<double> async_position_state_;
      std::atomic<double> async_velocity_state_;
      std::atomic<double> async_effort_state_;
      std::atomic<double> async_position_command_;
      std::atomic<double> async_velocity_command_;
      std::atomic<double> async_effort_command_;
      std::atomic<bool> position_interface_running_;
      std::atomic<bool> velocity_interface_running_;
      std::atomic<bool> effort_interface_running_;

  };

} //namespace rmd_hardware_interface

#endif //RMD_HARDWARE_INTERFACE_HPP_