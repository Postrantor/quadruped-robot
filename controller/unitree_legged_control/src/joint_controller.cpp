/**
 * @author GPT4-o
 * @brief Implementation of the Joint Controller for Unitree Legged Robots
 * @date 2024-06-23 16:55:22
 */

#include "unitree_legged_control/joint_controller.hpp"

namespace unitree_legged_control {

UnitreeJointController::UnitreeJointController() {
  memset(&last_cmd_, 0, sizeof(unitree_msgs::msg::MotorCmd));
  memset(&last_state_, 0, sizeof(unitree_msgs::msg::MotorState));
  memset(&servo_cmd_, 0, sizeof(ServoCmd));
}

UnitreeJointController::~UnitreeJointController() {}

controller_interface::return_type UnitreeJointController::init(const std::string &controller_name) {
  auto node = get_node();
  node->declare_parameter("joint", rclcpp::ParameterValue(""));
  std::string joint_name = node->get_parameter("joint").as_string();

  if (joint_name.empty()) {
    RCLCPP_ERROR(node->get_logger(), "No joint given in parameter file");
    return controller_interface::return_type::ERROR;
  }

  joint_ = robot_hardware_->get_joint_handle(joint_name, "position");

  sub_ft_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "joint_wrench", 1, std::bind(&UnitreeJointController::setTorqueCB, this, std::placeholders::_1));
  sub_cmd_ = node->create_subscription<unitree_msgs::msg::MotorCmd>(
      "command", 20, std::bind(&UnitreeJointController::setCommandCB, this, std::placeholders::_1));
  pub_state_ = node->create_publisher<unitree_msgs::msg::MotorState>("state", 1);

  return controller_interface::return_type::OK;
}

controller_interface::return_type UnitreeJointController::on_activate(const rclcpp_lifecycle::State &) {
  command_buffer_.writeFromNonRT(unitree_msgs::msg::MotorCmd{});
  return controller_interface::return_type::OK;
}

controller_interface::return_type UnitreeJointController::on_deactivate(const rclcpp_lifecycle::State &) {
  return controller_interface::return_type::OK;
}

controller_interface::return_type UnitreeJointController::update() {
  auto cmd = *(command_buffer_.readFromRT());
  double current_pos = joint_.get_position();
  double current_vel = computeVel(current_pos, last_state_.q, last_state_.dq, 0.01);  // duration placeholder

  if (cmd.mode == PMSM) {
    servo_cmd_.pos = cmd.q;
    positionLimits(servo_cmd_.pos);
    servo_cmd_.posStiffness = cmd.Kp;
    servo_cmd_.vel = cmd.dq;
    velocityLimits(servo_cmd_.vel);
    servo_cmd_.velStiffness = cmd.Kd;
    servo_cmd_.torque = cmd.tau;
    effortLimits(servo_cmd_.torque);
  } else if (cmd.mode == BRAKE) {
    servo_cmd_.posStiffness = 0;
    servo_cmd_.vel = 0;
    servo_cmd_.velStiffness = 0;
    servo_cmd_.torque = 0;
  }

  double calc_torque = computeTorque(current_pos, current_vel, servo_cmd_);
  effortLimits(calc_torque);

  joint_.set_command(calc_torque);

  last_state_.q = current_pos;
  last_state_.dq = current_vel;
  last_state_.tauEst = joint_.get_effort();

  pub_state_->publish(last_state_);

  return controller_interface::return_type::OK;
}

void UnitreeJointController::setTorqueCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  servo_cmd_.torque = msg->wrench.torque.x;
}

void UnitreeJointController::setCommandCB(const unitree_msgs::msg::MotorCmd::SharedPtr msg) {
  last_cmd_.mode = msg->mode;
  last_cmd_.q = msg->q;
  last_cmd_.Kp = msg->Kp;
  last_cmd_.dq = msg->dq;
  last_cmd_.Kd = msg->Kd;
  last_cmd_.tau = msg->tau;
  command_buffer_.writeFromNonRT(last_cmd_);
}

void UnitreeJointController::positionLimits(double &position) {
  // Example: Clamping logic
  position = clamp(position, -1.0, 1.0);  // Use appropriate joint limits
}

void UnitreeJointController::velocityLimits(double &velocity) {
  // Example: Clamping logic
  velocity = clamp(velocity, -10.0, 10.0);  // Use appropriate joint limits
}

void UnitreeJointController::effortLimits(double &effort) {
  // Example: Clamping logic
  effort = clamp(effort, -100.0, 100.0);  // Use appropriate joint limits
}

void UnitreeJointController::setGains(
    const double &p,
    const double &i,
    const double &d,
    const double &i_max,
    const double &i_min,
    const bool &antiwindup) {
  pid_controller_.setGains(p, i, d, i_max, i_min, antiwindup);
}

void UnitreeJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup) {
  pid_controller_.getGains(p, i, d, i_max, i_min, antiwindup);
}

void UnitreeJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min) {
  bool dummy;
  pid_controller_.getGains(p, i, d, i_max, i_min, dummy);
}

}  // namespace unitree_legged_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(unitree_legged_control::UnitreeJointController, controller_interface::ControllerInterface)
