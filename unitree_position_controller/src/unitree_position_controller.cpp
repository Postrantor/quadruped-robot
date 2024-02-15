// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "lifecycle_msgs/msg/state.hpp"

#include "unitree_position_controller/unitree_position_controller.hpp"

namespace
{
// @zhiqi.jia, 从这里sub数据
/*
  ros2 topic pub --rate 30 /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "
  twist:
    linear:
      x: 0.7
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 1.0"
*/
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";  // sub
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace unitree_position_controller
{
using namespace std::chrono_literals;

UnitreePositionController::UnitreePositionController() : controller_interface::ControllerInterface()
{
}

const char* UnitreePositionController::feedback_type() const
{
  // @zhiqi.jia, 依据控制类型不同，返回不同的状态作为反馈
  return params_.position_feedback ? hardware_interface::HW_IF_POSITION : hardware_interface::HW_IF_VELOCITY;
}

/**
 * @brief 0. 初始化各种资源
 * @details 1.创建param_listener node; 2.从yaml文件中解析参数;
 * read from `ros2_control/<description>.ros2_control.xacro`
 * read from `config/*controller.yaml`
 * @return controller_interface::CallbackReturn
 */
controller_interface::CallbackReturn UnitreePositionController::on_init()
{
  try
  {
    // create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception& e)
  {
    // @zhiqi.jia, use RCLCPP_ERROR?
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 2. command_interface, type `ALL`，`INDIVIDUAL` 和 `NONE`
 * 将方法全部注册到基类对应的成员变量中，通过 share memory 的方式和 hardware 方面通信
 * @return controller_interface::InterfaceConfiguration
 * return `<joint_name>/<interface_type>`
 */
controller_interface::InterfaceConfiguration UnitreePositionController::command_interface_configuration() const
{
  /*
    <joint name="${prefix}left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    HW_IF_POSITION[] = "position";
    HW_IF_VELOCITY[] = "velocity";
    HW_IF_ACCELERATION[] = "acceleration";
    HW_IF_EFFORT[] = "effort";
  */
  std::vector<std::string> conf_names;
  for (const auto& joint_name : params_.joint_ll_0_name)
  {
    // `<joint_name>/<interface_type>`
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  // for (const auto& joint_name : params_.left_wheel_names)
  // {
  //   // `<joint_name>/<interface_type>`
  //   conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  // }
  // for (const auto& joint_name : params_.right_wheel_names)
  // {
  //   conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  // }
  return { controller_interface::interface_configuration_type::INDIVIDUAL, conf_names };
}

// 2. state_interface 同上
controller_interface::InterfaceConfiguration UnitreePositionController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto& joint_name : params_.joint_ll_0_name)
  {
    // FIXME: (@zhiqi.jia), 这里只有一个类型的接口？
    // 应该是根据控制方式不同，反馈不同的状态
    conf_names.push_back(joint_name + "/" + feedback_type());
  }
  return { controller_interface::interface_configuration_type::INDIVIDUAL, conf_names };
}

/**
 * @brief 4. 从硬件接口获取状态，并下发命令到硬件
 *
 * @param time 当前时间，time - last_time
 * @param period 周期
 * @return controller_interface::return_type
 */
controller_interface::return_type UnitreePositionController::update(const rclcpp::Time& time,
                                                                    const rclcpp::Duration& period)
{
  rclcpp::Logger logger = get_node()->get_logger();

  // @zhiqi.jia是否刹车
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    // @zhiqi.jia, 刹车, wheel_handle.velocity.get().set_value(0.0);
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  // 1. --- --- --- sub command
  // 这里是 sub 到的数据，应该是通过终端直接pub的
  // 由 velocity_command_subscriber_ 接收数据
  std::shared_ptr<geometry_msgs::msg::TwistStamped> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);
  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  // @zhiqi.jia 超时处理 -> 刹车
  // brake if cmd_vel has timeout, override the stored command
  const auto age_of_last_command = time - last_command_msg->header.stamp;
  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }
  // command may be limited further by SpeedLimit, without affecting the stored twist command
  geometry_msgs::msg::TwistStamped command = *last_command_msg;
  double& linear_command = command.twist.linear.x;
  double& angular_command = command.twist.angular.z;

  // @zhiqi.jia, 在 on_config() 中有？没见到用呀？
  previous_update_timestamp_ = time;  // (no use)

  // apply (possibly new) multipliers:
  const double wheel_separation = params_.wheel_separation_multiplier * params_.wheel_separation;
  const double left_wheel_radius = params_.left_wheel_radius_multiplier * params_.wheel_radius;
  const double right_wheel_radius = params_.right_wheel_radius_multiplier * params_.wheel_radius;

  // 2. --- --- --- read()->motor?, calc state
  // 由反馈或命令计算的里程计的信息, odometry_
  if (params_.open_loop)
  {
    // 开环控制？直接拿command计算里程信息
    odometry_.updateOpenLoop(linear_command, angular_command, time);
  }
  else
  {
    // 通过feedback计算里程信息
    double left_feedback_mean = 0.0;
    double right_feedback_mean = 0.0;
    for (size_t index = 0; index < static_cast<size_t>(params_.wheels_per_side); ++index)
    {
      // 获取轮子的位置反馈值
      // `registered_left_wheel_handles_` 就是 controller_interface<->hardware_interface的匹配
      // 所以，应该这里 read()->motor?
      const double left_feedback = registered_left_wheel_handles_[index].feedback.get().get_value();
      const double right_feedback = registered_right_wheel_handles_[index].feedback.get().get_value();

      if (std::isnan(left_feedback) || std::isnan(right_feedback))
      {
        RCLCPP_ERROR(logger, "Either the left or right wheel %s is invalid for index [%zu]", feedback_type(), index);
        return controller_interface::return_type::ERROR;
      }

      // 累计求和？
      left_feedback_mean += left_feedback;
      right_feedback_mean += right_feedback;
    }
    left_feedback_mean /= params_.wheels_per_side;
    right_feedback_mean /= params_.wheels_per_side;

    if (params_.position_feedback)
    {
      odometry_.update(left_feedback_mean, right_feedback_mean, time);
    }
    else
    {
      // 速度积分，得到里程计信息
      odometry_.updateFromVelocity(left_feedback_mean * left_wheel_radius * period.seconds(),
                                   right_feedback_mean * right_wheel_radius * period.seconds(), time);
    }
  }
  // 获得航向角？(no use)
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  // 3. --- --- --- publish state
  // rviz会接受 publish odometry msg
  bool should_publish = false;  // TODO(@zhiqi.jia), use timer?
  try
  {
    if (previous_publish_timestamp_ + publish_period_ < time)
    {
      previous_publish_timestamp_ += publish_period_;
      should_publish = true;
    }
  }
  catch (const std::runtime_error&)
  {
    // Handle exceptions when the time source changes and initialize publish
    // timestamp
    previous_publish_timestamp_ = time;
    should_publish = true;
  }
  if (should_publish)
  {
    if (realtime_odometry_publisher_->trylock())
    {
      auto& odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = time;
      odometry_message.pose.pose.position.x = odometry_.getX();
      odometry_message.pose.pose.position.y = odometry_.getY();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
      odometry_message.twist.twist.linear.x = odometry_.getLinear();
      odometry_message.twist.twist.angular.z = odometry_.getAngular();
      realtime_odometry_publisher_->unlockAndPublish();
    }

    // for /tf
    if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
    {
      auto& transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = time;
      transform.transform.translation.x = odometry_.getX();
      transform.transform.translation.y = odometry_.getY();
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  // 4. --- --- --- publish limit command(twist)
  // publish limit command, but twist, not wheel command
  // just for ros, not for hardware
  // @zhiqi.jia, 通过存储 v_current, v_last, v_second, 对 dot_q, ddot_q, dddot_q 进行限制
  auto& last_command = previous_commands_.back().twist;
  auto& second_to_last_command = previous_commands_.front().twist;
  limiter_linear_.limit(linear_command, last_command.linear.x, second_to_last_command.linear.x, period.seconds());
  limiter_angular_.limit(angular_command, last_command.angular.z, second_to_last_command.angular.z, period.seconds());
  previous_commands_.pop();
  previous_commands_.emplace(command);  // update queue，limit_command not last_command_msg
  // publish limited velocity
  if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
  {
    auto& limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = time;
    limited_velocity_command.twist = command.twist;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }

  // 5. --- --- --- calc each wheel velocities, write()->motor
  // 这里需要紧接着将limit command分解为两侧轮子的速度信息，之后直接写到电机中write()->motor
  // controller_manager<->hardware_interface通过share memory 通信
  // 这里分解也应该转移到另一个文件中？使得主体的逻辑简洁？
  const double velocity_left = (linear_command - angular_command * wheel_separation / 2.0) / left_wheel_radius;
  const double velocity_right = (linear_command + angular_command * wheel_separation / 2.0) / right_wheel_radius;
  // set wheels velocities: -> write()->motor?
  for (size_t index = 0; index < static_cast<size_t>(params_.wheels_per_side); ++index)
  {
    // 这里就是 controller_interface<->hardware_interface的接口
    registered_left_wheel_handles_[index].velocity.get().set_value(velocity_left);
    registered_right_wheel_handles_[index].velocity.get().set_value(velocity_right);
  }

  // --- --- --- motor
  // 所以作为一个 demo，就直接给 hardware_interface 通信

  return controller_interface::return_type::OK;
}

/**
 * @brief 1. 创建 publisher_, subscriber_
 *
 * @return controller_interface::CallbackReturn
 */
controller_interface::CallbackReturn UnitreePositionController::on_configure(const rclcpp_lifecycle::State&)
{
  rclcpp::Logger logger = get_node()->get_logger();

  // --- --- --- update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  // check parameter
  if (params_.left_wheel_names.size() != params_.right_wheel_names.size())
  {
    RCLCPP_ERROR(logger,
                 "The number of left wheels [%zu] and the number of right "
                 "wheels [%zu] are different",
                 params_.left_wheel_names.size(), params_.right_wheel_names.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.left_wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "Wheel names parameters are empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  // calc parameter
  const double wheel_separation = params_.wheel_separation_multiplier * params_.wheel_separation;
  const double left_wheel_radius = params_.left_wheel_radius_multiplier * params_.wheel_radius;
  const double right_wheel_radius = params_.right_wheel_radius_multiplier * params_.wheel_radius;

  // set yaml parameter to controller
  odometry_.setWheelParams(wheel_separation, left_wheel_radius, right_wheel_radius);
  odometry_.setVelocityRollingWindowSize(params_.velocity_rolling_window_size);

  cmd_vel_timeout_ = std::chrono::milliseconds{ static_cast<int>(params_.cmd_vel_timeout * 1000.0) };
  publish_limited_velocity_ = params_.publish_limited_velocity;
  use_stamped_vel_ = params_.use_stamped_vel;

  // --- --- --- command limit
  limiter_linear_ = SpeedLimiter(
      params_.linear.x.has_velocity_limits, params_.linear.x.has_acceleration_limits, params_.linear.x.has_jerk_limits,
      params_.linear.x.min_velocity, params_.linear.x.max_velocity, params_.linear.x.min_acceleration,
      params_.linear.x.max_acceleration, params_.linear.x.min_jerk, params_.linear.x.max_jerk);
  limiter_angular_ =
      SpeedLimiter(params_.angular.z.has_velocity_limits, params_.angular.z.has_acceleration_limits,
                   params_.angular.z.has_jerk_limits, params_.angular.z.min_velocity, params_.angular.z.max_velocity,
                   params_.angular.z.min_acceleration, params_.angular.z.max_acceleration, params_.angular.z.min_jerk,
                   params_.angular.z.max_jerk);

  // --- --- --- reset all config
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  // left and right sides are both equal at this point
  params_.wheels_per_side = params_.left_wheel_names.size();

  // --- --- --- initialize pub command
  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
        DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(
            limited_velocity_publisher_);
  }

  // --- --- --- initialize command subscriber
  const geometry_msgs::msg::TwistStamped empty_twist;
  // fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);
  received_velocity_msg_ptr_.set(std::make_shared<geometry_msgs::msg::TwistStamped>(empty_twist));
  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
        DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) -> void {
          if (!subscriber_is_active_)
          {
            RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
          }
          if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
          {
            RCLCPP_WARN_ONCE(get_node()->get_logger(),
                             "Received TwistStamped with zero timestamp, "
                             "setting it to current "
                             "time, this message will only be shown once");
            msg->header.stamp = get_node()->get_clock()->now();
          }
          received_velocity_msg_ptr_.set(std::move(msg));
        });
  }
  else
  {
    velocity_command_unstamped_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
          if (!subscriber_is_active_)
          {
            RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
          }

          // Write fake header in the stored stamped command
          std::shared_ptr<Twist> twist_stamped;
          received_velocity_msg_ptr_.get(twist_stamped);
          twist_stamped->twist = *msg;
          twist_stamped->header.stamp = get_node()->get_clock()->now();
        });
  }

  // --- --- --- limit the publication on the topics /odom and /tf
  publish_rate_ = params_.publish_rate;
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  // --- --- --- initialize odometry publisher and messasge
  odometry_publisher_ =
      get_node()->create_publisher<nav_msgs::msg::Odometry>(DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher_);
  // append the tf prefix if there is one
  std::string tf_prefix = "";
  if (params_.tf_frame_prefix_enable)
  {
    if (params_.tf_frame_prefix != "")
    {
      tf_prefix = params_.tf_frame_prefix;
    }
    else
    {
      tf_prefix = std::string(get_node()->get_namespace());
    }

    if (tf_prefix == "/")
    {
      tf_prefix = "";
    }
    else
    {
      tf_prefix = tf_prefix + "/";
    }
  }
  const auto odom_frame_id = tf_prefix + params_.odom_frame_id;
  const auto base_frame_id = tf_prefix + params_.base_frame_id;
  //
  auto& odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_frame_id;
  odometry_message.child_frame_id = base_frame_id;
  // initialize odom values zeros
  odometry_message.twist = geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);
  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
  }

  // --- --- --- initialize transform publisher and message
  odometry_transform_publisher_ =
      get_node()->create_publisher<tf2_msgs::msg::TFMessage>(DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(odometry_transform_publisher_);
  // keeping track of odom and base_link transforms only
  auto& odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

  // --- --- --- time
  previous_update_timestamp_ = get_node()->get_clock()->now();

  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 将关节和对应的接口关联起来
 * @details controller_interface <-> hardware_interface
 * @return controller_interface::CallbackReturn, ==> `registered_handles_`
 */
controller_interface::CallbackReturn UnitreePositionController::on_activate(const rclcpp_lifecycle::State&)
{
  const auto left_result = configure_side("left", params_.left_wheel_names, registered_left_wheel_handles_);
  const auto right_result = configure_side("right", params_.right_wheel_names, registered_right_wheel_handles_);

  // check
  if (left_result == controller_interface::CallbackReturn::ERROR ||
      right_result == controller_interface::CallbackReturn::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (registered_left_wheel_handles_.empty() || registered_right_wheel_handles_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Either left wheel interfaces, right wheel interfaces are non "
                 "existent");
    return controller_interface::CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreePositionController::on_deactivate(const rclcpp_lifecycle::State&)
{
  subscriber_is_active_ = false;
  if (!is_halted)
  {
    halt();
    is_halted = true;
  }
  registered_left_wheel_handles_.clear();
  registered_right_wheel_handles_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreePositionController::on_cleanup(const rclcpp_lifecycle::State&)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreePositionController::on_error(const rclcpp_lifecycle::State&)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreePositionController::on_shutdown(const rclcpp_lifecycle::State&)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

bool UnitreePositionController::reset()
{
  odometry_.resetOdometry();

  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);

  registered_left_wheel_handles_.clear();
  registered_right_wheel_handles_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

void UnitreePositionController::halt()
{
  const auto halt_wheels = [](auto& wheel_handles) {
    for (const auto& wheel_handle : wheel_handles)
    {
      // 直接通过 controller_interface->hardware_interface 设置硬件对象的值
      wheel_handle.velocity.get().set_value(0.0);
    }
  };

  halt_wheels(registered_left_wheel_handles_);
  halt_wheels(registered_right_wheel_handles_);
}

// 3.
controller_interface::CallbackReturn UnitreePositionController::configure_side(
    const std::string& side, const std::vector<std::string>& wheel_names, std::vector<WheelHandle>& registered_handles)
{
  rclcpp::Logger logger = get_node()->get_logger();

  if (wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "No '%s' wheel names specified", side.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // 匹配 controller_interfaces <-> hardware_interfaces
  // controller_interface/include/controller_interface/controller_interface_base.hpp:243:
  // std::vector<hardware_interface::LoanedStateInterface> state_interfaces_;
  // --- --- ---
  // <joint name="${prefix}left_wheel_joint">
  //   <command_interface name="velocity"/>
  //   <state_interface name="position"/>
  //   <state_interface name="velocity"/>
  // </joint>
  registered_handles.reserve(wheel_names.size());
  for (const auto& wheel_name : wheel_names)
  {
    const auto interface_name = feedback_type();
    // 在这个函数中进行配置的关系？command_interface_configuration()
    // `<joint_name>/<interface_type>`
    // --- --- ---
    // // 在 controller_interfaces 会提供对应的方法，load对应的interfaces
    // void ControllerInterfaceBase::assign_interfaces(
    //   std::vector<hardware_interface::LoanedCommandInterface> && command_interfaces,
    //   std::vector<hardware_interface::LoanedStateInterface> && state_interfaces)
    // {
    //   command_interfaces_ = std::forward<decltype(command_interfaces)>(command_interfaces);
    //   state_interfaces_ = std::forward<decltype(state_interfaces)>(state_interfaces);
    // }
    // // controller_manager 会调用assign，从而获得对应的interface
    // controller->assign_interfaces(std::move(command_loans), std::move(state_loans));
    const auto state_handle = std::find_if(                    //
        state_interfaces_.cbegin(),                            //
        state_interfaces_.cend(),                              //
        [&wheel_name, &interface_name](const auto& interface)  //
        { return interface.get_prefix_name() == wheel_name && interface.get_interface_name() == interface_name; });
    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    // 同上
    const auto command_handle =
        std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&wheel_name](const auto& interface) {
          return interface.get_prefix_name() == wheel_name &&
                 interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
        });
    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(WheelHandle{ std::ref(*state_handle), std::ref(*command_handle) });
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace unitree_position_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(unitree_position_controller::UnitreePositionController,
                            controller_interface::ControllerInterface)
