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

#ifndef UNITREE_POSITION_CONTROLLER__UNITREE_POSITION_CONTROLLER_HPP_
#define UNITREE_POSITION_CONTROLLER__UNITREE_POSITION_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "unitree_position_controller/odometry.hpp"
#include "unitree_position_controller/speed_limiter.hpp"
#include "unitree_position_controller/visibility_control.h"

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "unitree_position_controller_parameters.hpp"

namespace unitree_position_controller
{
class UnitreePositionController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  UNITREE_POSITION_CONTROLLER_PUBLIC
  UnitreePositionController();

  // @zhiqi.jia, interface
  UNITREE_POSITION_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  UNITREE_POSITION_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // @zhiqi.jia update()
  UNITREE_POSITION_CONTROLLER_PUBLIC
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // @zhiqi.jia, lifecycle interface
  UNITREE_POSITION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  UNITREE_POSITION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  UNITREE_POSITION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  UNITREE_POSITION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  UNITREE_POSITION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  UNITREE_POSITION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  UNITREE_POSITION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

protected:
  // 这里的值是直接从硬件中 read() write() 通过进程内通信(share memory)获得
  // update() 中计算得到的两轮的速度指令，是直接set到这里
  // update() 中获得位置信息也是直接从这里get
  struct WheelHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
  };

  const char* feedback_type() const;
  controller_interface::CallbackReturn configure_side(const std::string& side,
                                                      const std::vector<std::string>& wheel_names,
                                                      std::vector<WheelHandle>& registered_handles);

  std::vector<WheelHandle> registered_left_wheel_handles_;
  std::vector<WheelHandle> registered_right_wheel_handles_;

  // parameters from ros for unitree_position_controller
  // @zhiqi.jia, generate code
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  // timeout to consider cmd_vel commands old
  // @zhiqi.jia, could read from yaml
  std::chrono::milliseconds cmd_vel_timeout_{ 500 };

  // @zhiqi.jia, algorithm，包含了计算里程计方法
  Odometry odometry_;

  // @zhiqi.jia, create odom publisher
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher_ = nullptr;
  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_odometry_transform_publisher_ =
      nullptr;

  // @zhiqi.jia, command subscriber
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_unstamped_subscriber_ = nullptr;
  realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::TwistStamped>> received_velocity_msg_ptr_{ nullptr };
  std::queue<geometry_msgs::msg::TwistStamped> previous_commands_;  // last two commands

  // @zhiqi.jia, algorithm, speed limiters
  SpeedLimiter limiter_linear_;
  SpeedLimiter limiter_angular_;
  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> limited_velocity_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>
      realtime_limited_velocity_publisher_ = nullptr;

  // publish rate limiter, @zhiqi.jia, use timer?
  rclcpp::Time previous_update_timestamp_{ 0 };
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{ 0, 0, RCL_CLOCK_UNINITIALIZED };
  bool is_halted = false;
  bool use_stamped_vel_ = true;
  bool reset();
  void halt();
};
}  // namespace unitree_position_controller
#endif  // UNITREE_POSITION_CONTROLLER__UNITREE_POSITION_CONTROLLER_HPP_
