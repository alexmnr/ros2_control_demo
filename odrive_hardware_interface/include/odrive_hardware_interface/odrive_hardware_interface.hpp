#ifndef ODRIVE_HARDWARE_INTERFACE__ODRIVE_HARDWARE_INTERFACE_HPP_
#define ODRIVE_HARDWARE_INTERFACE__ODRIVE_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "odrive_base/socket_can.hpp"
#include "odrive_base/socket_can.hpp"
#include "odrive_base/can_helpers.hpp"
#include "odrive_base/can_simple_messages.hpp"
#include "odrive_base/odrive_enums.h"

namespace odrive_hardware_interface
{
  class ODriveHardwareInterface : public hardware_interface::SystemInterface
  {
    enum Modes {               // Command Interfaces
      IDLE,                    // None or any other combination
      TORQUE_PASSTHROUGH,      // Effort
      VELOCITY_RAMPED,         // Velocity
      VELOCITY_PASSTHROUGH,    // Velocity & Effort
      POSITION_TRAJECTORY,     // Position
      POSITION_FILTERED,       // Position & Velocity
      POSITION_PASSTHROUGH,    // Position & Velocity & Effort
    };
    struct Joint {
      // Constructor
      explicit Joint(const hardware_interface::ComponentInfo & joint_info);
      // required parameters
      std::string name;
      uint8_t can_id;
      // optional parameters
      double motor_velocity_limit = std::numeric_limits<double>::quiet_NaN();
      double motor_current_limit = std::numeric_limits<double>::quiet_NaN();
      double position_p_gain = std::numeric_limits<double>::quiet_NaN();
      double velocity_p_gain = std::numeric_limits<double>::quiet_NaN();
      double velocity_i_gain = std::numeric_limits<double>::quiet_NaN();
      double input_filter_bandwidth = std::numeric_limits<double>::quiet_NaN();
      double trajectory_vel_limit = std::numeric_limits<double>::quiet_NaN();
      double trajectory_accel_limit = std::numeric_limits<double>::quiet_NaN();
      double trajectory_decel_limit = std::numeric_limits<double>::quiet_NaN();
      double trajectory_inertia = std::numeric_limits<double>::quiet_NaN();
      // command variables
      double position_command;
      double velocity_command;
      double effort_command;
      // state variables
      double position_state;
      double velocity_state;
      double effort_state;
      double effort_target;
      // Active command interface trackers
      bool is_position_active = false;
      bool is_velocity_active = false;
      bool is_effort_active = false;
      // odrive parameters
      uint32_t odrive_error = 1;
      uint8_t odrive_state = 0;
      uint8_t odrive_procedure_result = 0;
      uint8_t odrive_trajectory_done = 0;
      uint32_t input_vel_scale = 1000;
      uint32_t input_torque_scale = 1000;
      std::string fw_version = "";
      std::string hw_version = "";
      int mode = 0;
      bool ready = false;
      // Joint functions
      void send_commands();
      void switch_mode_if_necessary();
      void wait_till_ready();
      bool check_version();
      void clear_error();
      void request_encoder_feedback();
      void request_torques_feedback();
      void set_motor_limits();
      void set_trajectory_limits();
      void set_gains();
      void set_mode();
      std::string get_odrive_error_string(uint32_t error);
      std::string get_odrive_state_string(uint8_t state);
      // CAN
      SocketCanIntf* can_interface_;
      void on_can_msg(const can_frame& frame);
      void on_heartbeat_msg(const can_frame& frame);
      void on_version_msg(const can_frame& frame);
      void on_encoder_feedback_msg(const can_frame& frame);
      void on_torque_feedback_msg(const can_frame& frame);
      // CAN send template
      template <typename T>
        void send(const T& msg, bool rtr = false) {
          struct can_frame frame;
          frame.can_id = can_id << 5 | msg.cmd_id;
          if (rtr) {
            frame.can_id |= CAN_RTR_FLAG;
            frame.can_dlc = 0;
          } else {
            frame.can_dlc = msg.msg_length;
            msg.encode_buf(frame.data);
          }
          can_interface_->send_can_frame(frame);
        }
    };

    public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;
    hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;

    private:
    // required parameters
    std::string can_interface_name_;

    // CAN
    SocketCanIntf can_interface_;
    EpollEventLoop event_loop_;
    void on_can_msg(const can_frame& frame);
    void clear_all_errors();

    // actuators
    std::vector<Joint> joints_;
  };

}  // namespace odrive_hardware_interface

#endif  // ODRIVE_HARDWARE_INTERFACE__ODRIVE_HARDWARE_INTERFACE_HPP_
