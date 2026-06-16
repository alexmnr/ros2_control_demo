#include "odrive_pid_controller/odrive_pid_controller.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace odrive_pid_controller
{
  // --- on init ---
  controller_interface::CallbackReturn ODrivePidController::on_init() {
    try {
      param_listener = std::make_shared<ParamListener>(get_node());
      params = param_listener->get_params();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("ODrivePidController"), "[INIT] Exception thrown when reading parameters: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  // --- on configure ---
  controller_interface::CallbackReturn ODrivePidController::on_configure(const rclcpp_lifecycle::State &) {
    // Loop through joints
    for (const auto & [joint_name, joint_gains] : params.gains.joints_map) {
      // Create Joint object
      joints.emplace_back(joint_name, joint_gains);
      RCLCPP_ERROR(rclcpp::get_logger("ODrivePidController"), "[CONFIGURE] Create Joint '%s'", joint_name.c_str());
    }

    // Create listener
    joints_cmd_sub = this->get_node()->create_subscription<MultiDOFCommand>("odrive_pid_controller/reference", rclcpp::SystemDefaultsQoS(),
        [this](const MultiDOFCommand::SharedPtr msg) {
        rt_buffer.set(*msg);
        });
    return CallbackReturn::SUCCESS;
  }

  // --- on activate ---
  controller_interface::CallbackReturn ODrivePidController::on_activate(const rclcpp_lifecycle::State &) {
    // Loop through joints
    for (auto & joint : joints) {
      // assign state interfaces
      for (auto & state_interface : state_interfaces_) {
        if (state_interface.get_interface_name() == "position" && state_interface.get_prefix_name() == joint.name) {
          joint.position_state_handle = &state_interface;
        } else if (state_interface.get_interface_name() == "velocity" && state_interface.get_prefix_name() == joint.name) {
          joint.velocity_state_handle = &state_interface;
        }
      }
      // assign command interfaces
      for (auto & command_interface : command_interfaces_) {
        if (command_interface.get_interface_name() == "effort" && command_interface.get_prefix_name() == joint.name) {
          joint.effort_cmd_handle = &command_interface;
        }
      }
      // get initial reference value
      auto position_state_opt = joint.position_state_handle->get_optional();
      while (!position_state_opt.has_value());
      joint.position_reference = position_state_opt.value();
    }
    return CallbackReturn::SUCCESS;
  }

  // --- update ---
  controller_interface::return_type ODrivePidController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){
    // Update references from latest msg
    get_latest_references();
    // loop through joints
    for (auto& joint : joints) {
      // get current states from harware
      if (!joint.position_state_handle || !joint.velocity_state_handle || !joint.effort_cmd_handle) {
        continue; 
      }
      auto position_state_opt = joint.position_state_handle->get_optional();
      auto velocity_state_opt = joint.velocity_state_handle->get_optional();

      if (position_state_opt.has_value() && velocity_state_opt.has_value()) {
        // get current states
        double position_state = position_state_opt.value();
        double velocity_state = velocity_state_opt.value();
        // calculate pid output
        double position_error = joint.position_reference - position_state;
        double temp_output = joint.position_pid->compute_command(position_error, period);
        double velocity_error = temp_output - velocity_state;
        double effort_command = joint.velocity_pid->compute_command(velocity_error, period);
        // write effort value
        if (!joint.effort_cmd_handle->set_value(effort_command)) {
          RCLCPP_WARN(rclcpp::get_logger("ODrivePidController"), "Failed to set effort command interface value for joint '%s'", joint.name.c_str());
        }
      }
    }
    return controller_interface::return_type::OK;
  }

  // --- on deactivate ---
  controller_interface::CallbackReturn ODrivePidController::on_deactivate(const rclcpp_lifecycle::State &) {
    return CallbackReturn::SUCCESS;
  }

  // --- command interface configuration ---
  controller_interface::InterfaceConfiguration ODrivePidController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    std::vector<std::string> names_;
    for (const auto& joint : joints) {
      names_.push_back(joint.name + "/effort");
    }
    command_interfaces_config.names = names_;
    return command_interfaces_config;
  }

  // --- state interface configuration ---
  controller_interface::InterfaceConfiguration ODrivePidController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    std::vector<std::string> names_;
    for (const auto& joint : joints) {
      names_.push_back(joint.name + "/position");
      names_.push_back(joint.name + "/velocity");
    }
    state_interfaces_config.names = names_;
    return state_interfaces_config;
  }

  // --- get latest references ---
  void ODrivePidController::get_latest_references(){
    auto reference_op = rt_buffer.try_get();
    if (reference_op.has_value())
    {
      MultiDOFCommand reference_msg = reference_op.value();
      int i = 0;
      for (std::string dof_names : reference_msg.dof_names) {
        // search for joint with same name
        auto it = std::find_if(joints.begin(), joints.end(), [&](const Joint& joint) {
            return joint.name == dof_names;
            });
        // Check if the joint was found
        if (it != joints.end()) {
          it->position_reference = reference_msg.values[i];
        } else {
          RCLCPP_WARN(rclcpp::get_logger("ODrivePidController"), "Joint of name '%s' not recognized!", dof_names.c_str());
        }
        i++;
      }
    }
  }
}  // namespace odrive_pid_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(odrive_pid_controller::ODrivePidController, controller_interface::ControllerInterface)
