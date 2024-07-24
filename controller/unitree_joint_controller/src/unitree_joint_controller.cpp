/**
 * @brief controller for unitree motor
 * @date 2024-06-24 09:37:38
 * @copyright Copyright (c) 2024
 */

#include <iomanip>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "unitree_msgs/msg/motor_cmd.hpp"
#include "unitree_msgs/msg/motor_state.hpp"

#include "unitree_joint_controller/unitree_joint_controller.hpp"

namespace {
/**
  ros2 topic pub --rate 10 /unitree_base_controller/cmd_vel geometry_msgs/msg/TwistStamped
  "twist:
    linear: x: 0.7 y: 0.0 z: 0.0
    angular: x: 0.0 y: 0.0 z: 1.0"
*/
constexpr auto DEFAULT_DESIRED_CMD_TOPIC = "~/desired_cmd";    // sub
constexpr auto DEFAULT_REAL_CMD_TOPIC = "~/control_cmd";       // pub
constexpr auto DEFAULT_TARGET_STATE_TOPIC = "~/target_state";  // pub
}  // namespace

namespace unitree_joint_controller {

using namespace std::chrono_literals;

controller_interface::InterfaceConfiguration UnitreeJointController::command_interface_configuration() const {
  std::vector<std::string> conf_names;
  for (const auto &joint_name : params_.joint_name) {
    // `<joint_name>/<interface_type>`
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration UnitreeJointController::state_interface_configuration() const {
  std::vector<std::string> conf_names;
  for (const auto &joint_name : params_.joint_name) {
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

/**
 * @brief on_init()
 * @param params_ get parameters from controller.yaml
 * @return controller_interface::CallbackReturn
 */
controller_interface::CallbackReturn UnitreeJointController::on_init() {
  try {
    // create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(LOGGER, "exception thrown during init stage with message: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief config all member variable form params_
 * @return controller_interface::CallbackReturn
 */
controller_interface::CallbackReturn UnitreeJointController::on_configure(const rclcpp_lifecycle::State &) {
  // update parameters if its have changed
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO_STREAM(LOGGER, "parameters were updated");
  }

  // reset all config
  if (!reset()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // fill last two desired states with default constructed commands
  const unitree_msgs::msg::MotorCmd empty_cmd;
  previous_desired_cmd_.emplace(empty_cmd);
  previous_desired_cmd_.emplace(empty_cmd);

  rb_desired_cmd_.set(std::make_shared<unitree_msgs::msg::MotorCmd>(empty_cmd));
  subscribe_desired_cmd_ = get_node()->create_subscription<unitree_msgs::msg::MotorCmd>(
      DEFAULT_DESIRED_CMD_TOPIC,    // topic name
      rclcpp::SystemDefaultsQoS(),  // qos config
      [this](const std::shared_ptr<unitree_msgs::msg::MotorCmd> msg) -> void {
        if (!subscriber_is_active_) {
          RCLCPP_WARN_STREAM(LOGGER, "can't accept new commands. subscriber is inactive");
          return;
        }
        rb_desired_cmd_.set(std::move(msg));
      });

  // initialize publisher
  publish_real_cmd_ = get_node()->create_publisher<unitree_msgs::msg::MotorCmd>(
      DEFAULT_REAL_CMD_TOPIC,  //
      rclcpp::SystemDefaultsQoS());
  publish_target_state_ = get_node()->create_publisher<unitree_msgs::msg::MotorState>(
      DEFAULT_TARGET_STATE_TOPIC,  //
      rclcpp::SystemDefaultsQoS());

  // timer
  publish_rate_ = params_.publish_rate;
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
  desired_cmd_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.desired_cmd_timeout * 1000.0)};
  previous_update_timestamp_ = get_node()->get_clock()->now();

  // arithmetic
  pid_p_ = params_.pid.p;
  pid_i_ = params_.pid.i;
  pid_d_ = params_.pid.d;

  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief setting the flag indicates successful activation
 * @return controller_interface::CallbackReturn
 */
controller_interface::CallbackReturn UnitreeJointController::on_activate(const rclcpp_lifecycle::State &) {
  // get hardware_interface
  const auto result = get_joint_handle(params_.joint_name, registered_joint_handles_);
  if (result == controller_interface::CallbackReturn::ERROR) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (registered_joint_handles_.empty()) {
    RCLCPP_ERROR_STREAM(LOGGER, "either left wheel interfaces, right wheel interfaces are non existent");
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG_STREAM(LOGGER, "subscriber and publisher are now active.");

  is_halted_ = false;
  subscriber_is_active_ = true;

  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief reactivate controller
 * @return controller_interface::CallbackReturn
 */
controller_interface::CallbackReturn UnitreeJointController::on_deactivate(const rclcpp_lifecycle::State &) {
  subscriber_is_active_ = false;
  if (!is_halted_) {
    halt();
    is_halted_ = true;
  }
  registered_joint_handles_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief reset params
 * @return controller_interface::CallbackReturn
 */
controller_interface::CallbackReturn UnitreeJointController::on_cleanup(const rclcpp_lifecycle::State &) {
  if (!reset()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  rb_desired_cmd_.set(std::make_shared<unitree_msgs::msg::MotorCmd>());

  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief set error
 * @return controller_interface::CallbackReturn
 */
controller_interface::CallbackReturn UnitreeJointController::on_error(const rclcpp_lifecycle::State &) {
  if (!reset()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief unload controller
 * @return controller_interface::CallbackReturn
 */
controller_interface::CallbackReturn UnitreeJointController::on_shutdown(const rclcpp_lifecycle::State &) {
  // do something
  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief update controller
 *     - sub cmd => `desired_cmd`
 *     - read()->motor => `target_state`
 *     - calc `control_cmd=pid_controller(desired_cmd, target_state, time, period)`
 *     - get `read_cmd=safety(control_cmd)`
 *     - write()->motor <= `read_cmd`
 * @param time
 * @param period
 * @return controller_interface::return_type
 */
controller_interface::return_type UnitreeJointController::update(
    const rclcpp::Time &time,  //
    const rclcpp::Duration & /*period*/) {
  // 0. check lifecycle state
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted_) {
      halt();
      is_halted_ = true;
    }
    return controller_interface::return_type::OK;
  }

  // 1. get `desired state` from rb_desired_cmd_
  std::shared_ptr<unitree_msgs::msg::MotorCmd> last_desired_cmd;
  rb_desired_cmd_.get(last_desired_cmd);
  if (last_desired_cmd == nullptr) {
    RCLCPP_WARN_STREAM(LOGGER, "desired state message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }
  // brake, if `desired state` has timeout, override the stored command
  if ((time - last_desired_cmd->header.stamp) > desired_cmd_timeout_) {
    last_desired_cmd->mode = BRAKE;
  }
  // command may be limited further by safety, without affecting the stored twist command.
  unitree_msgs::msg::MotorCmd desired_cmd = *last_desired_cmd;  // copy desired_cmd
  // unitree_msgs::msg::MotorCmd real_cmd;                         // for publish
  unitree_msgs::msg::MotorState target_state;  // for publish

  // 2. read `target_state` from hardware_interface
  // FIXME(@zhiqi.jia) :: 针对unitree motor应该使用上次write()中获取的结果？
  for (size_t i = 0; i < params_.joint_name.size(); ++i) {
    const double feedback_position = registered_joint_handles_[i].feedback_position.get().get_value();
    const double feedback_velocity = registered_joint_handles_[i].feedback_velocity.get().get_value();

    if (std::isnan(feedback_position) || std::isnan(feedback_velocity)) {
      RCLCPP_ERROR_STREAM(LOGGER, "either the position or velocity joint is invalid for index " << i);
      return controller_interface::return_type::ERROR;
    }
    target_state.q = feedback_position;
    target_state.dq = feedback_velocity;
  }

  // TODO(@zhiqi.jia) :: should add limited
  if (desired_cmd.mode == PMSM) {
    servo_cmd_.pos = desired_cmd.q;
    servo_cmd_.posStiffness = desired_cmd.k_q;
    servo_cmd_.vel = desired_cmd.dq;
    servo_cmd_.velStiffness = desired_cmd.k_dq;
    servo_cmd_.torque = desired_cmd.tau;  // ?
  } else if (desired_cmd.mode == BRAKE) {
    servo_cmd_.posStiffness = 0;
    servo_cmd_.vel = 0;
    servo_cmd_.velStiffness = 0;
    servo_cmd_.torque = 0;
  }
  // use unitree_leged_sdk calc torque
  // simulation?
  target_state.tau = computeTorque(target_state.q, target_state.dq, servo_cmd_);

  // 4. publish `target_state` and safety `real_cmd`
  bool should_publish = false;
  try {
    // handle exceptions when the time source changes and initialize publish timestamp
    if (previous_publish_timestamp_ + publish_period_ < time) {
      previous_publish_timestamp_ += publish_period_;
      should_publish = true;
    }
  } catch (const std::runtime_error &) {
    previous_publish_timestamp_ = time;
    should_publish = true;
  }
  if (should_publish) {
    publish_real_cmd_->publish(desired_cmd);  // real_cmd=sub_cmd
    publish_target_state_->publish(target_state);
    // and write()->motor
    for (size_t i = 0; i < params_.joint_name.size(); ++i) {
      registered_joint_handles_[i].command_velocity.get().set_value(desired_cmd.dq);
      RCLCPP_INFO_STREAM(
          LOGGER, "update()->write " << params_.joint_name[i] << " to hardware;" << "\n\tand pub to topic: " << DEFAULT_REAL_CMD_TOPIC << " and " << DEFAULT_TARGET_STATE_TOPIC);
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn UnitreeJointController::get_joint_handle(
    const std::vector<std::string> &joints_name,  //
    std::vector<JointHandle> &registered_handles) {
  if (joints_name.empty()) {
    RCLCPP_ERROR_STREAM(LOGGER, "no joint names specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  // controller_interfaces <-> hardware_interfaces
  registered_handles.reserve(joints_name.size());
  for (const auto &joint_name : joints_name) {
    // find state_position_handle
    const auto state_position_handle = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),  //
        [&joint_name](const auto &interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
        });
    if (state_position_handle == state_interfaces_.cend()) {
      RCLCPP_ERROR_STREAM(LOGGER, "unable to obtain joint state handle for " << joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    // find state_velocity_handle
    const auto state_velocity_handle = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),  //
        [&joint_name](const auto &interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
        });
    if (state_velocity_handle == state_interfaces_.cend()) {
      RCLCPP_ERROR_STREAM(LOGGER, "unable to obtain joint state handle for " << joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    // find command_velocity_handle
    const auto command_velocity_handle = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),  //
        [&joint_name](const auto &interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
        });
    if (command_velocity_handle == command_interfaces_.end()) {
      RCLCPP_ERROR_STREAM(LOGGER, "unable to obtain joint command handle for " << joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    // return
    registered_handles.emplace_back(JointHandle{
        std::ref(*command_velocity_handle),  //
        std::ref(*state_position_handle),    //
        std::ref(*state_velocity_handle),    //
    });
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

bool UnitreeJointController::reset() {
  // reset pid controller parameter
  // empty queue
  std::queue<unitree_msgs::msg::MotorCmd> empty;
  std::swap(previous_desired_cmd_, empty);
  // clear
  registered_joint_handles_.clear();
  // deactivate
  subscriber_is_active_ = false;
  subscribe_desired_cmd_.reset();
  rb_desired_cmd_.set(nullptr);
  // FIXME(@zhiqi.jia), should be true?
  is_halted_ = false;
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
