#include "odrive_pid_controller/odrive_pid_controller.hpp"

namespace odrive_pid_controller 
{
  ODrivePidController::Joint::Joint(const std::string & joint_name, const odrive_pid_controller::Params::Gains::MapJoints & joint_gains) {
    name = joint_name;
    // create pids
    control_toolbox::AntiWindupStrategy anti_windup_strategy_;
    anti_windup_strategy_.set_type("none");
    position_pid = std::make_shared<control_toolbox::Pid>(
        joint_gains.position.p,
        joint_gains.position.i,
        joint_gains.position.d,
        joint_gains.position.max_output,
        joint_gains.position.min_output,
        anti_windup_strategy_
        );
    velocity_pid = std::make_shared<control_toolbox::Pid>(
        joint_gains.velocity.p,
        joint_gains.velocity.i,
        joint_gains.velocity.d,
        joint_gains.velocity.max_output,
        joint_gains.velocity.min_output,
        anti_windup_strategy_
        );
  }
}  // namespace odrive_pid_controller
