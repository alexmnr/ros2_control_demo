#include "odrive_hardware_interface/odrive_hardware_interface.hpp"

namespace odrive_hardware_interface
{
  // --- initialize joint object from parameters ---
  ODriveHardwareInterface::Joint::Joint(const hardware_interface::ComponentInfo & joint_info) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT] Creating joint with name: %s", joint_info.name.c_str());

    // Check for correct size of command and state interfaces
    if (joint_info.command_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT] Joint '%s' has %zu command interfaces. 3 expected.", joint_info.name.c_str(), joint_info.command_interfaces.size());
      throw std::runtime_error("Invalid number of command interfaces");
    }
    if (joint_info.state_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT] Joint '%s' has %zu state interfaces. 3 expected.", joint_info.name.c_str(), joint_info.state_interfaces.size());
      throw std::runtime_error("Invalid number of state interfaces");
    }

    // Assign name
    name = joint_info.name;

    // Parse required parameters
    const auto & params = joint_info.parameters;
    if (!params.count("can_id")) {
      RCLCPP_FATAL(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT] Parameter can_id has to be specified!");
      throw std::runtime_error("Missing required parameter: can_id");
    }
    can_id = (uint8_t)std::stoi(params.at("can_id"));
    // Parse optional parameters 
    if (params.count("motor_velocity_limit")) motor_velocity_limit = std::stod(params.at("motor_velocity_limit"));
    if (params.count("motor_current_limit")) motor_current_limit = std::stod(params.at("motor_current_limit"));
    if (params.count("position_p_gain")) position_p_gain = std::stod(params.at("position_p_gain"));
    if (params.count("velocity_p_gain")) velocity_p_gain = std::stod(params.at("velocity_p_gain"));
    if (params.count("velocity_i_gain")) velocity_i_gain = std::stod(params.at("velocity_i_gain"));
    if (params.count("input_filter_bandwidth")) input_filter_bandwidth = std::stod(params.at("input_filter_bandwidth"));
    if (params.count("trajectory_vel_limit")) trajectory_vel_limit = std::stod(params.at("trajectory_vel_limit"));
    if (params.count("trajectory_accel_limit")) trajectory_accel_limit = std::stod(params.at("trajectory_accel_limit"));
    if (params.count("trajectory_decel_limit")) trajectory_decel_limit = std::stod(params.at("trajectory_decel_limit"));
    if (params.count("trajectory_inertia")) trajectory_inertia = std::stod(params.at("trajectory_inertia"));
    // print info
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT]   can-id: %d", can_id);
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT]   velocity-limit: %.1f", motor_velocity_limit);
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT]   current-limit: %.1f", motor_current_limit);
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT]   position-p-gain: %.1f", position_p_gain);
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT]   velocity-p-gain: %.1f", velocity_p_gain);
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT]   velocity-i-gain: %.1f", velocity_i_gain);
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT]   input_filter_bandwith: %.1f", input_filter_bandwidth);
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT]   trajectory_vel_limit: %.1f", trajectory_vel_limit);
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT]   trajectory_accel_limit: %.1f", trajectory_accel_limit);
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT]   trajectory_descel_limit: %.1f", trajectory_decel_limit);
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[INIT]   trajectory_inertia: %.1f", trajectory_inertia);
  }

  // --- handle can messages ---
  void ODriveHardwareInterface::Joint::on_can_msg(const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    switch (cmd) {
      case Get_Encoder_Estimates_msg_t::cmd_id: {
        on_encoder_feedback_msg(frame);
      } break;
      case Get_Torques_msg_t::cmd_id: {
        on_torque_feedback_msg(frame);
      } break;
      case Heartbeat_msg_t::cmd_id: {
        on_heartbeat_msg(frame);
      } break;
      case Get_Version_msg_t::cmd_id: {
        on_version_msg(frame);
      } break;
    }
  }

  // --- wait till ready ---
  void ODriveHardwareInterface::Joint::wait_till_ready() {
    // Check if all joints are ready
    for (int i = 0; i <= 5; i++) {
      Get_Torques_msg_t get_torques_msg;
      Get_Encoder_Estimates_msg_t get_encoder_estimages_msg;
      send(get_torques_msg, true);
      send(get_encoder_estimages_msg, true);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      while (can_interface_->read_nonblocking()) {}
      if (odrive_error == 0 && position_state != 0) {
        position_command = position_state;
        ready = true;
        return;
      } 
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[ACTIVATE] Waiting for Joint '%s' to be ready...", name.c_str());
    }
  }

  // --- check version ---
  bool ODriveHardwareInterface::Joint::check_version() {
    Get_Version_msg_t msg;
    send(msg, true);
    for (int i = 0; i <= 5; i++) {
      while (can_interface_->read_nonblocking()); // repeat until CAN interface has no more messages
      if (hw_version.length() != 0 && fw_version.length() != 0) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[CONFIGURE] Found: Joint '%s' can-id '%d' with fw-version '%s' and hw-version '%s'", name.c_str(), can_id, fw_version.c_str(), hw_version.c_str());
        if (fw_version != "0:6:11" || hw_version != "5:2:0") {
          RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "[CONFIGURE] Joint '%s' with can-id '%d' is running the wrong version! Please update to 0:6:11 and 5:2:0.", name.c_str(), can_id);
          return false;
        } else {
          return true;
        }
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "[CONFIG] Could not reach motor with can-id '%d'! Check CAN connection!", can_id);
    return false;
  }

  // --- on encoder feedback msg ---
  void ODriveHardwareInterface::Joint::on_encoder_feedback_msg(const can_frame& frame) {
    if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
      RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", Get_Encoder_Estimates_msg_t::cmd_id);
      return;
    }
    Get_Encoder_Estimates_msg_t msg;
    msg.decode_buf(frame.data);
    position_state = msg.Pos_Estimate * (2 * M_PI);
    velocity_state = msg.Vel_Estimate * (2 * M_PI);
  }

  // --- on torque feedback msg ---
  void ODriveHardwareInterface::Joint::on_torque_feedback_msg(const can_frame& frame) {
    if (frame.can_dlc < Get_Torques_msg_t::msg_length) {
      RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", Get_Torques_msg_t::cmd_id);
      return;
    }
    Get_Torques_msg_t msg;
    msg.decode_buf(frame.data);
    effort_target = msg.Torque_Target;
    effort_state = msg.Torque_Estimate;
  }

  // --- on version msg ---
  void ODriveHardwareInterface::Joint::on_version_msg(const can_frame& frame) {
    Get_Version_msg_t msg;
    msg.decode_buf(frame.data);
    hw_version = std::to_string(msg.Hw_Version_Major) + ":" + std::to_string(msg.Hw_Version_Minor) + ":" + std::to_string(msg.Hw_Version_Variant);
    fw_version = std::to_string(msg.Fw_Version_Major) + ":" + std::to_string(msg.Fw_Version_Minor) + ":" + std::to_string(msg.Fw_Version_Revision);
  }

  // --- on heartbeat msg ---
  void ODriveHardwareInterface::Joint::on_heartbeat_msg(const can_frame& frame) {
    if (frame.can_dlc < Heartbeat_msg_t::msg_length) {
      RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", Heartbeat_msg_t::cmd_id);
      return;
    }
    Heartbeat_msg_t msg;
    msg.decode_buf(frame.data);
    odrive_error = msg.Axis_Error;
    odrive_state = msg.Axis_State;
    odrive_procedure_result = msg.Procedure_Result;
    odrive_trajectory_done = msg.Trajectory_Done_Flag;
    if (odrive_error != 0) {
      ready = false;
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "Joint '%s' with can-id '%d' has error '%s' and state '%s'", name.c_str(), can_id, get_odrive_error_string(odrive_error).c_str(), get_odrive_state_string(odrive_state).c_str());
    }
  }

  // --- clear error ---
  void ODriveHardwareInterface::Joint::clear_error() {
    Clear_Errors_msg_t msg;
    msg.Identify = 0;
    send(msg);
  }

  // --- request encoder feedback ---
  void ODriveHardwareInterface::Joint::request_encoder_feedback() {
    Get_Encoder_Estimates_msg_t get_encoder_estimages_msg;
    send(get_encoder_estimages_msg, true);
  }
  
  // --- request torques feedback ---
  void ODriveHardwareInterface::Joint::request_torques_feedback() {
    Get_Torques_msg_t get_torques_msg;
    send(get_torques_msg, true);
  }

  // --- set motor limits ---
  void ODriveHardwareInterface::Joint::set_motor_limits() {
    if (std::isnan(motor_velocity_limit) || std::isnan(motor_current_limit)) {
      return;
    }
    // Set Velocity Limit
    Set_Limits_msg_t msg;
    if (motor_velocity_limit != 0) {
      msg.Velocity_Limit = (float)motor_velocity_limit;
    } else {
      msg.Velocity_Limit = std::numeric_limits<float>::infinity();
    }
    // Set Soft Current Limit
    if (motor_current_limit != 0) {
      msg.Current_Limit = (float)motor_current_limit;
    } else {
      msg.Current_Limit = std::numeric_limits<float>::infinity();
    }
    send(msg);
  }

  // --- set trajectory limits ---
  void ODriveHardwareInterface::Joint::set_trajectory_limits() {
    // Set Velocity Limit
    if (!std::isnan(trajectory_vel_limit)) {
      Set_Traj_Vel_Limit_msg_t vel_limit_msg;
      vel_limit_msg.Traj_Vel_Limit = (float)trajectory_vel_limit;
      send(vel_limit_msg);
    }
    // Set Acceleration Limit
    if (!std::isnan(trajectory_accel_limit) && !std::isnan(trajectory_decel_limit)) {
      Set_Traj_Accel_Limits_msg_t acc_limit_msg;
      acc_limit_msg.Traj_Accel_Limit = (float)trajectory_accel_limit;
      acc_limit_msg.Traj_Decel_Limit = (float)trajectory_decel_limit;
      send(acc_limit_msg);
    }
    // Set Inertia
    if (!std::isnan(trajectory_inertia)) {
      Set_Traj_Inertia_msg_t inertia_msg;
      inertia_msg.Traj_Inertia = (float)trajectory_inertia;
      send(inertia_msg);
    }
  }

  // --- set gains ---
  void ODriveHardwareInterface::Joint::set_gains() {
    // Set Position Gains
    if (!std::isnan(position_p_gain)) {
      Set_Pos_Gain_msg_t pos_msg;
      pos_msg.Pos_Gain = (float)position_p_gain;
      send(pos_msg);
    }
    // Set Velocity Gains
    if (!std::isnan(velocity_p_gain) && !std::isnan(velocity_i_gain)) {
      Set_Vel_Gains_msg_t vel_msg;
      vel_msg.Vel_Gain = (float)velocity_p_gain;
      vel_msg.Vel_Integrator_Gain = (float)velocity_i_gain;
      send(vel_msg);
    }
  }

  // --- set mode ---
  void ODriveHardwareInterface::Joint::set_mode() {
    // IDLE
    if (mode == Modes::IDLE) {
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_IDLE;
      send(set_axis_state_msg);
      RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[MODE SWITCH] Joint '%s' switched to mode: IDLE", name.c_str());
    // TORQUE_PASSTHROUGH 
    } else if (mode == Modes::TORQUE_PASSTHROUGH) {
      Set_Controller_Mode_msg_t set_controller_mode_msg;
      set_controller_mode_msg.Control_Mode = ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL;
      set_controller_mode_msg.Input_Mode = ODriveInputMode::INPUT_MODE_PASSTHROUGH;
      send(set_controller_mode_msg);
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
      send(set_axis_state_msg);
      RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[MODE SWITCH] Joint '%s' switched to mode: TORQUE_CONTROL", name.c_str());
    // VELOCITY RAMPED
    } else if (mode == Modes::VELOCITY_RAMPED) {
      Set_Controller_Mode_msg_t set_controller_mode_msg;
      set_controller_mode_msg.Control_Mode = ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL;
      set_controller_mode_msg.Input_Mode = ODriveInputMode::INPUT_MODE_VEL_RAMP;
      send(set_controller_mode_msg);
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
      send(set_axis_state_msg);
      RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[MODE SWITCH] Joint '%s' switched to mode: VELOCITY_RAMPED", name.c_str());
    // VELOCITY PASSTHROUGH
    } else if (mode == Modes::VELOCITY_PASSTHROUGH) {
      Set_Controller_Mode_msg_t set_controller_mode_msg;
      set_controller_mode_msg.Control_Mode = ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL;
      set_controller_mode_msg.Input_Mode = ODriveInputMode::INPUT_MODE_PASSTHROUGH;
      send(set_controller_mode_msg);
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
      send(set_axis_state_msg);
      RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[MODE SWITCH] Joint '%s' switched to mode: VELOCITY_PASSTHROUGH", name.c_str());
    // POSITION TRAJECOTRY
    } else if (mode == Modes::POSITION_TRAJECTORY) {
      Set_Controller_Mode_msg_t set_controller_mode_msg;
      set_controller_mode_msg.Control_Mode = ODriveControlMode::CONTROL_MODE_POSITION_CONTROL;
      set_controller_mode_msg.Input_Mode = ODriveInputMode::INPUT_MODE_TRAP_TRAJ;
      send(set_controller_mode_msg);
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
      send(set_axis_state_msg);
      RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[MODE SWITCH] Joint '%s' switched to mode: POSITION_TRAJECTORY", name.c_str());
    // POSITION FILTERED
    } else if (mode == Modes::POSITION_FILTERED) {
      Set_Controller_Mode_msg_t set_controller_mode_msg;
      set_controller_mode_msg.Control_Mode = ODriveControlMode::CONTROL_MODE_POSITION_CONTROL;
      set_controller_mode_msg.Input_Mode = ODriveInputMode::INPUT_MODE_POS_FILTER;
      send(set_controller_mode_msg);
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
      send(set_axis_state_msg);
      RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[MODE SWITCH] Joint '%s' switched to mode: POSITION_FILTERED", name.c_str());
    // POSITION PASSTHROUGH
    } else if (mode == Modes::POSITION_PASSTHROUGH) {
      Set_Controller_Mode_msg_t set_controller_mode_msg;
      set_controller_mode_msg.Control_Mode = ODriveControlMode::CONTROL_MODE_POSITION_CONTROL;
      set_controller_mode_msg.Input_Mode = ODriveInputMode::INPUT_MODE_PASSTHROUGH;
      send(set_controller_mode_msg);
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
      send(set_axis_state_msg);
      RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "[MODE SWITCH] Joint '%s' switched to mode: POSITION_PASSTHROUGH", name.c_str());
    // ELSE
    } else {
      RCLCPP_FATAL(rclcpp::get_logger("ODriveHardwareInterface"), "[MODE SWITCH] Joint '%s' received unrecognized mode: %d", name.c_str(), mode);
    }
  }

  // --- get odrive state string ---
  std::string ODriveHardwareInterface::Joint::get_odrive_state_string(uint8_t state) {
    switch (state) {
      case 0: return "UNDEFINED";
      case 1: return "IDLE";
      case 2: return "STARTUP_SEQUENCE";
      case 3: return "FULL_CALIBRATION_SEQUENCE";
      case 4: return "MOTOR_CALIBRATION";
      case 6: return "ENCODER_INDEX_SEARCH";
      case 7: return "ENCODER_OFFSET_CALIBRATION";
      case 8: return "CLOSED_LOOP_CONTROL";
      case 9: return "LOCKIN_SPIN";
      case 10: return "ENCODER_DIR_FIND";
      case 11: return "HOMING";
      case 12: return "ENCODER_HALL_POLARITY_CALIBRATION";
      case 13: return "ENCODER_HALL_PHASE_CALIBRATION";
      case 14: return "ANTICOGGING_CALIBRATION";
      default: return "UNKNOWN_STATE (" + std::to_string(state) + ")";
    }
  }

  // --- get odrive error string ---
  std::string ODriveHardwareInterface::Joint::get_odrive_error_string(uint32_t error) {
    switch (error) {
        case 0x00000000: return "ODRIVE_ERROR_NONE";
        case 0x00000001: return "ODRIVE_ERROR_INITIALIZING";
        case 0x00000002: return "ODRIVE_ERROR_SYSTEM_LEVEL";
        case 0x00000004: return "ODRIVE_ERROR_TIMING_ERROR";
        case 0x00000008: return "ODRIVE_ERROR_MISSING_ESTIMATE";
        case 0x00000010: return "ODRIVE_ERROR_BAD_CONFIG";
        case 0x00000020: return "ODRIVE_ERROR_DRV_FAULT";
        case 0x00000040: return "ODRIVE_ERROR_MISSING_INPUT";
        case 0x00000100: return "ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE";
        case 0x00000200: return "ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE";
        case 0x00000400: return "ODRIVE_ERROR_DC_BUS_OVER_CURRENT";
        case 0x00000800: return "ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT";
        case 0x00001000: return "ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION";
        case 0x00002000: return "ODRIVE_ERROR_MOTOR_OVER_TEMP";
        case 0x00004000: return "ODRIVE_ERROR_INVERTER_OVER_TEMP";
        case 0x00008000: return "ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION";
        case 0x00010000: return "ODRIVE_ERROR_POSITION_LIMIT_VIOLATION";
        case 0x01000000: return "ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED";
        case 0x02000000: return "ODRIVE_ERROR_ESTOP_REQUESTED";
        case 0x04000000: return "ODRIVE_ERROR_SPINOUT_DETECTED";
        case 0x08000000: return "ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED";
        case 0x10000000: return "ODRIVE_ERROR_THERMISTOR_DISCONNECTED";
        case 0x40000000: return "ODRIVE_ERROR_CALIBRATION_ERROR";
        default: {
            std::stringstream ss;
            ss << "UNKNOWN_ERROR_CODE (0x" << std::setfill('0') << std::setw(8) << std::hex << error << ")";
            return ss.str();
        }
    }
  }
}  // namespace odrive_hardware_interface
