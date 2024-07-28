/**
 * @brief
 * @date 2024-07-29 19:59:20
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

#include "unitree_position_controller/unitree_position_controller.hpp"

namespace {
constexpr auto DEFAULT_DESIRED_STATE_TOPIC = "~/desired_state";  // sub
constexpr auto DEFAULT_TARGET_STATE_TOPIC = "~/target_state";    // pub
}  // namespace

namespace unitree_position_controller {
using namespace std::chrono_literals;

controller_interface::InterfaceConfiguration UnitreePositionController::command_interface_configuration() const {
  std::vector<std::string> conf_names;
  for (const auto &joint_name : params_.joint_name) {
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration UnitreePositionController::state_interface_configuration() const {
  std::vector<std::string> conf_names;
  for (const auto &joint_name : params_.joint_name) {
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

CallbackReturn UnitreePositionController::on_init() {
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

CallbackReturn UnitreePositionController::on_configure(const rclcpp_lifecycle::State &) {
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
  const geometry_msgs::msg::TwistStamped empty_twist{};
  previous_desired_states_.emplace(empty_twist);
  previous_desired_states_.emplace(empty_twist);

  // fill last two desired states with default constructed commands
  received_desired_state_ptr_.set(std::make_shared<geometry_msgs::msg::TwistStamped>(empty_twist));
  subscribe_desired_state_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      DEFAULT_DESIRED_STATE_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) -> void {
        if (!subscriber_is_active_) {
          RCLCPP_WARN_STREAM(LOGGER, "can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
          RCLCPP_WARN_ONCE(
              LOGGER,
              "received twiststamped with zero timestamp, "
              "setting it to current time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        received_desired_state_ptr_.set(std::move(msg));
      });

  // initialize publisher
  publishe_target_state_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
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

CallbackReturn UnitreePositionController::on_activate(const rclcpp_lifecycle::State &) {
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

CallbackReturn UnitreePositionController::on_deactivate(const rclcpp_lifecycle::State &) {
  subscriber_is_active_ = false;
  if (!is_halted) {
    halt();
    is_halted = true;
  }
  registered_joint_handles_.clear();
  return CallbackReturn::SUCCESS;
}

CallbackReturn UnitreePositionController::on_cleanup(const rclcpp_lifecycle::State &) {
  if (!reset()) {
    return CallbackReturn::ERROR;
  }
  received_desired_state_ptr_.set(std::make_shared<geometry_msgs::msg::TwistStamped>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn UnitreePositionController::on_error(const rclcpp_lifecycle::State &) {
  if (!reset()) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn UnitreePositionController::on_shutdown(const rclcpp_lifecycle::State &) {
  // do something
  return CallbackReturn::SUCCESS;
}

/**
 * @brief update controller
 * @param time
 * @param period
 * @return controller_interface::return_type
 */
controller_interface::return_type UnitreePositionController::update(
    const rclcpp::Time &time,  //
    const rclcpp::Duration & /*period*/) {
  // 0. check lifecycle state
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted) {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  // 1. sub `desired_state`
  std::shared_ptr<geometry_msgs::msg::TwistStamped> last_desired_state;
  received_desired_state_ptr_.get(last_desired_state);
  if (last_desired_state == nullptr) {
    RCLCPP_WARN_STREAM(LOGGER, "desired state message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  // brake, if desired_state has timeout, override the stored command
  if ((time - last_desired_state->header.stamp) > desired_state_timeout_) {
    last_desired_state->twist.linear.x = 0.0;
  }

  geometry_msgs::msg::TwistStamped target_state;  // for publish

  // 2. read `target_state` from hardware_interface
  for (size_t i = 0; i < params_.joint_name.size(); ++i) {
    const double feedback_position = registered_joint_handles_[i].feedback_position.get().get_value();
    const double feedback_velocity = registered_joint_handles_[i].feedback_velocity.get().get_value();

    if (std::isnan(feedback_position) || std::isnan(feedback_velocity)) {
      RCLCPP_ERROR_STREAM(LOGGER, "either the position or velocity joint is invalid for index " << i);
      return controller_interface::return_type::ERROR;
    }
    target_state.twist.angular.x = feedback_position;
    target_state.twist.angular.y = feedback_velocity;

    RCLCPP_INFO_STREAM(
        LOGGER, "update()->read(): \n\tfeedback_position: " << feedback_position
                                                            << "\n\tfeedback_velocity: " << feedback_velocity);
  }

  // 4. publish `target_state`
  bool should_publish = false;  // TODO(@zhiqi.jia), use timer?
  try {
    if (previous_publish_timestamp_ + publish_period_ < time) {
      previous_publish_timestamp_ += publish_period_;
      should_publish = true;
    }
  } catch (const std::runtime_error &) {
    previous_publish_timestamp_ = time;
    should_publish = true;
  }
  if (should_publish) {
    publishe_target_state_->publish(target_state);
    // and write()->motor
    for (size_t i = 0; i < params_.joint_name.size(); ++i) {
      registered_joint_handles_[i].command_velocity.get().set_value(last_desired_state->twist.linear.x);
      RCLCPP_INFO_STREAM(
          LOGGER, "update()->write() " << params_.joint_name[i] << "last_desired_state->twist.linear.x: "
                                       << last_desired_state->twist.linear.x << " to hardware");
    }
  }

  return controller_interface::return_type::OK;
}

CallbackReturn UnitreePositionController::get_joint_handle(
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

    // return
    registered_handles.emplace_back(JointHandle{
        std::ref(*command_velocity_handle),  //
        std::ref(*state_position_handle),    //
        std::ref(*state_velocity_handle),    //
    });
  }

  return CallbackReturn::SUCCESS;
}

// reset pid controller parameter
bool UnitreePositionController::reset() {
  // empty queue
  std::queue<geometry_msgs::msg::TwistStamped> empty;
  std::swap(previous_desired_states_, empty);

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

void UnitreePositionController::halt() {
  const auto halt_joints = [](auto &handles) {
    for (const auto &handle : handles) {
      handle.command_velocity.get().set_value(0.0);
    }
  };

  halt_joints(registered_joint_handles_);
}

}  // namespace unitree_position_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    unitree_position_controller::UnitreePositionController,  //
    controller_interface::ControllerInterface)
