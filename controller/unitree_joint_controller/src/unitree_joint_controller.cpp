/**
 * @brief
 * @date 2024-08-04 23:41:27
 * @copyright Copyright (c) 2024
 */

#include <iomanip>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "unitree_msgs/msg/motor_cmd.hpp"
#include "unitree_msgs/msg/motor_state.hpp"

#include "unitree_joint_controller/unitree_joint_controller.hpp"
#include "unitree_joint_controller/unitree_joint_control_tool.hpp"

namespace {
constexpr auto DEFAULT_DESIRED_STATE_TOPIC = "~/desired_state";  // sub
constexpr auto DEFAULT_TARGET_STATE_TOPIC = "~/target_state";    // pub
}  // namespace

namespace unitree_joint_controller {
using namespace std::chrono_literals;

controller_interface::InterfaceConfiguration UnitreeJointController::command_interface_configuration() const {
  std::vector<std::string> conf_names;
  for (const auto &joint_name : params_.joint_name) {
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration UnitreeJointController::state_interface_configuration() const {
  std::vector<std::string> conf_names;
  for (const auto &joint_name : params_.joint_name) {
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

CallbackReturn UnitreeJointController::on_init() {
  try {
    // create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(LOGGER, "exception thrown during init stage with message: " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn UnitreeJointController::on_configure(const rclcpp_lifecycle::State &) {
  // update parameters if its have changed
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO_STREAM(LOGGER, "parameters were updated");
  }

  // reset all config
  if (!reset()) {
    return CallbackReturn::ERROR;
  }

  // fill last two desired states with default constructed commands
  const unitree_msgs::msg::MotorState empty_sate{};
  previous_states_.emplace(empty_sate);
  previous_states_.emplace(empty_sate);

  // fill last two desired states with default constructed commands
  const unitree_msgs::msg::MotorCmd empty_desired_state{};
  received_desired_state_ptr_.set(std::make_shared<unitree_msgs::msg::MotorCmd>(empty_desired_state));
  subscribe_desired_state_ = get_node()->create_subscription<unitree_msgs::msg::MotorCmd>(
      DEFAULT_DESIRED_STATE_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<unitree_msgs::msg::MotorCmd> msg) -> void {
        if (!subscriber_is_active_) {
          RCLCPP_WARN_STREAM(LOGGER, "can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
          RCLCPP_WARN_ONCE(
              LOGGER,
              "received unitree stamped with zero timestamp, "
              "setting it to current time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        received_desired_state_ptr_.set(std::move(msg));
      });

  // initialize publisher
  publishe_target_state_ = get_node()->create_publisher<unitree_msgs::msg::MotorState>(
      DEFAULT_TARGET_STATE_TOPIC, rclcpp::SystemDefaultsQoS());

  // limit publish on the /odom and /tf
  publish_rate_ = params_.publish_rate;
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  // set parameter to class member
  desired_state_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.desired_state_timeout * 1000.0)};

  // time
  previous_update_timestamp_ = get_node()->get_clock()->now();

  return CallbackReturn::SUCCESS;
}

CallbackReturn UnitreeJointController::on_activate(const rclcpp_lifecycle::State &) {
  // get hardware_interface
  const auto result = get_joint_handle(params_.joint_name, registered_joint_handles_);
  if (result == CallbackReturn::ERROR) {
    return CallbackReturn::ERROR;
  }
  if (registered_joint_handles_.empty()) {
    RCLCPP_ERROR_STREAM(LOGGER, "either left wheel interfaces, right wheel interfaces are non existent");
    return CallbackReturn::ERROR;
  }

  // activate
  is_halted = false;
  subscriber_is_active_ = true;
  RCLCPP_DEBUG_STREAM(LOGGER, "subscriber and publisher are now active.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn UnitreeJointController::on_deactivate(const rclcpp_lifecycle::State &) {
  subscriber_is_active_ = false;
  if (!is_halted) {
    halt();
    is_halted = true;
  }
  registered_joint_handles_.clear();
  return CallbackReturn::SUCCESS;
}

CallbackReturn UnitreeJointController::on_cleanup(const rclcpp_lifecycle::State &) {
  if (!reset()) {
    return CallbackReturn::ERROR;
  }
  received_desired_state_ptr_.set(std::make_shared<unitree_msgs::msg::MotorCmd>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn UnitreeJointController::on_error(const rclcpp_lifecycle::State &) {
  if (!reset()) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn UnitreeJointController::on_shutdown(const rclcpp_lifecycle::State &) {
  // do something
  return CallbackReturn::SUCCESS;
}

/**
 * @brief update controller
 * @param time
 * @param period
 * @return controller_interface::return_type
 */
controller_interface::return_type UnitreeJointController::update(
    const rclcpp::Time &time,  //
    const rclcpp::Duration &period) {
  // 0. check lifecycle state
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted) {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  // 1. sub `desired_state`
  std::shared_ptr<unitree_msgs::msg::MotorCmd> last_desired_state;
  received_desired_state_ptr_.get(last_desired_state);
  if (last_desired_state == nullptr) {
    RCLCPP_WARN_STREAM(LOGGER, "desired state message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  // brake, if desired_state has timeout, override the stored command
  // if ((time - last_desired_state->header.stamp) > desired_state_timeout_) {
  //   last_desired_state->mode == BRAKE;
  //   RCLCPP_WARN_STREAM(LOGGER, "break, desired_state timeout.");
  // }

  if (last_desired_state->mode == BRAKE) {
    servo_cmd_.posStiffness = 0;
    servo_cmd_.vel = 0;
    servo_cmd_.velStiffness = 0;
    servo_cmd_.torque = 0;
    effort_limits(servo_cmd_.torque);
    RCLCPP_WARN_STREAM(LOGGER, "break, stop joint.");
  }

  // copy from unitree_sdk
  if (last_desired_state->mode == PMSM) {
    servo_cmd_.pos = last_desired_state->q;
    position_limits(servo_cmd_.pos);

    servo_cmd_.posStiffness = last_desired_state->k_q;
    if (fabs(last_desired_state->q - posStopF) < 0.00001) {
      servo_cmd_.posStiffness = 0;
    }

    servo_cmd_.vel = last_desired_state->dq;
    velocity_limits(servo_cmd_.vel);

    servo_cmd_.velStiffness = last_desired_state->k_dq;
    if (fabs(last_desired_state->dq - velStopF) < 0.00001) {
      servo_cmd_.velStiffness = 0;
    }

    servo_cmd_.torque = last_desired_state->tau;
    effort_limits(servo_cmd_.torque);

    RCLCPP_INFO_STREAM(LOGGER, "limit joint action.");
  }

  // 2. read `target_state` from hardware_interface
  for (size_t i = 0; i < params_.joint_name.size(); ++i) {
    target_state_.q = registered_joint_handles_[i].feedback_position.get().get_value();
    target_state_.dq = registered_joint_handles_[i].feedback_velocity.get().get_value();
    target_state_.tau_est = registered_joint_handles_[i].feedback_effort.get().get_value();

    if (std::isnan(target_state_.q) || std::isnan(target_state_.dq) || std::isnan(target_state_.tau_est)) {
      RCLCPP_ERROR_STREAM(LOGGER, "the joint position/velocity/effort is invalid for index " << i);
      return controller_interface::return_type::ERROR;
    }

    RCLCPP_INFO_STREAM(
        LOGGER, "\n" << params_.joint_name[i] << "\n\t- feedback_position: " << target_state_.q  //
                     << "\n\t- feedback_velocity: " << target_state_.dq
                     << "\n\t- feedback_effort: " << target_state_.tau_est);
  }

  // 3. 依据电机状态信息/许用值估算(微分)出电机受力状态信息
  current_pos_ = target_state_.q;
  current_vel_ = computeVel(current_pos_, (double)target_state_.q, (double)target_state_.dq, period.seconds());
  calc_torque_ = computeTorque(current_pos_, current_vel_, servo_cmd_);
  effort_limits(calc_torque_);

  for (size_t i = 0; i < params_.joint_name.size(); ++i) {
    registered_joint_handles_[i].command_effort.get().set_value(calc_torque_);
    RCLCPP_INFO_STREAM(LOGGER, "\n" << params_.joint_name[i] << "\n\t- control_effort: " << calc_torque_);
  }

  last_desired_state->q = current_pos_;
  last_desired_state->dq = current_vel_;
  last_desired_state->tau = calc_torque_;

  publishe_target_state_->publish(target_state_);

  return controller_interface::return_type::OK;
}

void UnitreeJointController::position_limits(double &position) {
  clamp(position, params_.position.limit_lower, params_.position.limit_upper);
}

void UnitreeJointController::velocity_limits(double &velocity) {
  clamp(velocity, params_.velocity.limit_lower, params_.velocity.limit_upper);
}

void UnitreeJointController::effort_limits(double &effort) {
  clamp(effort, params_.effort.limit_lower, params_.effort.limit_upper);
}

CallbackReturn UnitreeJointController::get_joint_handle(
    const std::vector<std::string> &joints_name,  //
    std::vector<JointHandle> &registered_handles) {
  if (joints_name.empty()) {
    RCLCPP_ERROR_STREAM(LOGGER, "no joint names specified");
    return CallbackReturn::ERROR;
  }

  // controller_interfaces <-> hardware_interfaces
  registered_handles.reserve(joints_name.size());
  for (const auto &joint_name : joints_name) {
    // find state_position_handle
    const auto state_position_handle = std::find_if(
        state_interfaces_.cbegin(),  //
        state_interfaces_.cend(),    //
        [&joint_name](const auto &interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
        });
    if (state_position_handle == state_interfaces_.cend()) {
      RCLCPP_ERROR_STREAM(LOGGER, "unable to obtain joint state handle for " << joint_name.c_str());
      return CallbackReturn::ERROR;
    }
    // find state_velocity_handle
    const auto state_velocity_handle = std::find_if(
        state_interfaces_.cbegin(),  //
        state_interfaces_.cend(),    //
        [&joint_name](const auto &interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
        });
    if (state_velocity_handle == state_interfaces_.cend()) {
      RCLCPP_ERROR_STREAM(LOGGER, "unable to obtain joint state handle for " << joint_name.c_str());
      return CallbackReturn::ERROR;
    }
    // find state_effort_handle
    const auto state_effort_handle = std::find_if(
        state_interfaces_.cbegin(),  //
        state_interfaces_.cend(),    //
        [&joint_name](const auto &interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
        });
    if (state_effort_handle == state_interfaces_.cend()) {
      RCLCPP_ERROR_STREAM(LOGGER, "unable to obtain joint state handle for " << joint_name.c_str());
      return CallbackReturn::ERROR;
    }
    // find command_position_handle
    const auto command_position_handle = std::find_if(
        command_interfaces_.begin(),  //
        command_interfaces_.end(),    //
        [&joint_name](const auto &interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
        });
    if (command_position_handle == command_interfaces_.end()) {
      RCLCPP_ERROR_STREAM(LOGGER, "unable to obtain joint command handle for " << joint_name.c_str());
      return CallbackReturn::ERROR;
    }
    // find command_velocity_handle
    const auto command_velocity_handle = std::find_if(
        command_interfaces_.begin(),  //
        command_interfaces_.end(),    //
        [&joint_name](const auto &interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
        });
    if (command_velocity_handle == command_interfaces_.end()) {
      RCLCPP_ERROR_STREAM(LOGGER, "unable to obtain joint command handle for " << joint_name.c_str());
      return CallbackReturn::ERROR;
    }
    // find command_effort_handle
    const auto command_effort_handle = std::find_if(
        command_interfaces_.begin(),  //
        command_interfaces_.end(),    //
        [&joint_name](const auto &interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
        });
    if (command_effort_handle == command_interfaces_.end()) {
      RCLCPP_ERROR_STREAM(LOGGER, "unable to obtain joint command handle for " << joint_name.c_str());
      return CallbackReturn::ERROR;
    }

    // return
    registered_handles.emplace_back(JointHandle{
        std::ref(*command_position_handle),  //
        std::ref(*command_velocity_handle),  //
        std::ref(*command_effort_handle),    //
        std::ref(*state_position_handle),    //
        std::ref(*state_velocity_handle),    //
        std::ref(*state_effort_handle),      //
    });
  }

  return CallbackReturn::SUCCESS;
}

// reset pid controller parameter
bool UnitreeJointController::reset() {
  // empty queue
  std::queue<unitree_msgs::msg::MotorState> empty;
  std::swap(previous_states_, empty);

  // clear
  registered_joint_handles_.clear();

  // deactivate
  subscriber_is_active_ = false;
  subscribe_desired_state_.reset();
  received_desired_state_ptr_.set(nullptr);

  // FIXME(@zhiqi.jia), should be true?
  is_halted = false;
  return true;
}

void UnitreeJointController::halt() {
  const auto halt_joints = [](auto &handles) {
    for (const auto &handle : handles) {
      handle.command_velocity.get().set_value(0.0);
    }
  };

  halt_joints(registered_joint_handles_);
}

}  // namespace unitree_joint_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    unitree_joint_controller::UnitreeJointController,  //
    controller_interface::ControllerInterface)
