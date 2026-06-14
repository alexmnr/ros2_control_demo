#include "odrive_hardware_interface/odrive_hardware_interface.hpp"

namespace odrive_hardware_interface
{
  // --- on_init ---
  hardware_interface::CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params) {
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Load hardware interface parameters
    can_interface_name_ = info_.hardware_parameters["can_interface"];
    if (can_interface_name_ != "") {
      RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT] CAN inteface: %s", can_interface_name_.c_str());
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT] No can interface provided!");
      return hardware_interface::CallbackReturn::FAILURE;
    }

    // Iterate through joints
    for (const auto & joint_info : info_.joints) {
      try {
        joints_.emplace_back(joint_info); 
      } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT] Failed to initialize joint: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // --- on_configure ---
  hardware_interface::CallbackReturn ODriveHardwareInterface::on_configure(const rclcpp_lifecycle::State &) {
    // connect to CAN
    if (!can_interface_.init(can_interface_name_, &event_loop_, std::bind(&ODriveHardwareInterface::on_can_msg, this, _1))) {
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "[CONFIGURE] Failed to initialize SocketCAN on %s", can_interface_name_.c_str());
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "[CONFIGURE] Have you run 'sudo ip link set up can0 type can bitrate 1000000'?");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[CONFIGURE] Succesfully initialized SocketCAN on %s", can_interface_name_.c_str());
    // Parse can interface to each joint
    for (auto& joint : joints_) {
      joint.can_interface_ = &can_interface_;
    }
    // Check if Motor connected and if version is correct
    for (auto& joint : joints_) {
      if (!joint.check_version()) {
        return CallbackReturn::ERROR;
      }
    }
    // clear all errors
    clear_all_errors();
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[CONFIGURE] Cleared all errors");

    // write parameters
    for (auto& joint : joints_) {
      joint.set_motor_limits();
      joint.set_trajectory_limits();
      joint.set_gains();
    }
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[CONFIGURE] Succesfully set parameters");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // --- on_activate ---
  hardware_interface::CallbackReturn ODriveHardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
    // Set all motors to IDLE
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[ACTIVATE] Changing mode to: IDLE");
    for (auto& joint : joints_) {
      joint.mode = Modes::IDLE;
      joint.set_mode();
    }
    // Wait for motors to be ready
    for (auto& joint : joints_) {
      joint.wait_till_ready();
      RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[ACTIVATE] Motor with can-id '%d' for joint '%s' is ready!", joint.can_id, joint.name.c_str());
    }
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[ACTIVATE] Hardware succesfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // --- on_deactivate ---
  hardware_interface::CallbackReturn ODriveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &) {
    // Configure all axis
    for (auto& joint : joints_) {
      joint.mode = Modes::IDLE;
      joint.set_mode();
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // --- on_cleanup ---
  hardware_interface::CallbackReturn ODriveHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // --- on_shutdown ---
  hardware_interface::CallbackReturn ODriveHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // --- on_error ---
  hardware_interface::CallbackReturn ODriveHardwareInterface::on_error(const rclcpp_lifecycle::State &) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // --- read ---
  hardware_interface::return_type ODriveHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
    return hardware_interface::return_type::OK;
  }

  // --- write ---
  hardware_interface::return_type ODriveHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
    return hardware_interface::return_type::OK;
  }

  // --- export_state_interfaces ---
  std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    return state_interfaces;
  }

  // --- export_command_interfaces ---
  std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    return command_interfaces;
  }

  // --- prepare_command_mode_switch ---
  hardware_interface::return_type ODriveHardwareInterface::prepare_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/, const std::vector<std::string> & /*stop_interfaces*/) {
    return hardware_interface::return_type::OK;
  }

  // --- perform_command_mode_switch ---
  hardware_interface::return_type ODriveHardwareInterface::perform_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/, const std::vector<std::string> & /*stop_interfaces*/) {
    return hardware_interface::return_type::OK;
  }

  // --- pass can messages to individual joints ---
  void ODriveHardwareInterface::on_can_msg(const can_frame& frame) {
    for (auto& joint : joints_) {
      if ((frame.can_id >> 5) == joint.can_id) {
        joint.on_can_msg(frame);
      }
    }
  }

  // --- clear all errors ---
  void ODriveHardwareInterface::clear_all_errors() {
    for (auto& joint : joints_) {
      joint.clear_error();
    }
  }
}  // namespace odrive_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(odrive_hardware_interface::ODriveHardwareInterface, hardware_interface::SystemInterface)
