/**
 * @brief controller for unitree motor
 * @date 2024-06-24 09:37:38
 * @copyright Copyright (c) 2024
 */

#ifndef UNITREE_JOINT_CONTROLLER__UNITREE_JOINT_CONTROLLER_HPP_
#define UNITREE_JOINT_CONTROLLER__UNITREE_JOINT_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <cmath>
#include <queue>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_publisher.h"

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "unitree_msgs/msg/motor_cmd.hpp"
#include "unitree_msgs/msg/motor_state.hpp"

#include "unitree_joint_controller/unitree_joint_control_tool.hpp"
#include "unitree_joint_controller_parameters.hpp"  // generate to build folder
#include "unitree_joint_controller/visibility_control.h"

namespace unitree_joint_controller {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("unitree_joint_controller");

class UnitreeJointController : public controller_interface::ControllerInterface {
public:
  /**
   * @brief Construct a new Unitree Joint Controller object
   * @details 这里采用的是 lifecycle node 的方式，所以创建、销毁pub/sub的操作需要在对应的api中实现
   * 构造函数中不再有任何实现
   */
  UNITREE_JOINT_CONTROLLER_PUBLIC
  UnitreeJointController() = default;

  /**
   * @brief 2. command_interface, type `ALL`，`INDIVIDUAL` 和 `NONE`
   * 将方法全部注册到基类对应的成员变量中，通过 share memory 的方式和 hardware
   * 方面通信
   * @return controller_interface::InterfaceConfiguration
   * return `<joint_name>/<interface_type>`
   */
  UNITREE_JOINT_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  // 2. state_interface 同上
  UNITREE_JOINT_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief 4. 从硬件接口获取状态，并下发命令到硬件
   * @param time 当前时间，time - last_time
   * @param period 周期
   * @return controller_interface::return_type
   */
  UNITREE_JOINT_CONTROLLER_PUBLIC
  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  /**
   * @brief 0. 初始化各种资源
   * @details 1.创建param_listener node; 2.从yaml文件中解析参数;
   *  read from `ros2_control/<description>.ros2_control.xacro`
   *  read from `config/_controller.yaml`
   *  这里仅仅做一些必要的初始化，比如构造一些临时的对象等
   *  对于 pub/sub 这类对象，作为类的成员函数中定义，在`on_configure()`中构造
   * @return controller_interface::CallbackReturn
   */
  UNITREE_JOINT_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief 1. 创建 publisher_, subscriber_
   * @return controller_interface::CallbackReturn
   */
  UNITREE_JOINT_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief 将关节和对应的接口关联起来
   * @details controller_interface <-> hardware_interface
   * @return controller_interface::CallbackReturn, ==> `registered_handles_`
   */
  UNITREE_JOINT_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  UNITREE_JOINT_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  UNITREE_JOINT_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  UNITREE_JOINT_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

  UNITREE_JOINT_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

protected:
  // subscribe motor_cmd as `desired state` set to motor
  bool subscriber_is_active_{false};  // for lifecycle config
  rclcpp::Subscription<unitree_msgs::msg::MotorCmd>::SharedPtr subscribe_desired_cmd_{nullptr};
  realtime_tools::RealtimeBox<std::shared_ptr<unitree_msgs::msg::MotorCmd>> rb_desired_cmd_{nullptr};
  std::queue<unitree_msgs::msg::MotorCmd> previous_desired_cmd_;  // the two most recent control commands
  // publish `real_cmd`
  rclcpp::Publisher<unitree_msgs::msg::MotorCmd>::SharedPtr publish_real_cmd_{nullptr};
  // publish `target_state`
  rclcpp::Publisher<unitree_msgs::msg::MotorState>::SharedPtr publish_target_state_{nullptr};

  // parameters from ros for unitree_joint_controller
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  // about timer
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_update_timestamp_{0};
  rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
  std::chrono::milliseconds desired_cmd_timeout_{500};  // timeout to refresh

  // arithmetic
  double pid_p_{0};
  double pid_i_{0};
  double pid_d_{0};

  // safety
  // SpeedLimiter limiter_linear_;
  // SpeedLimiter limiter_angular_;

  // unitree_msgs::msg::MotorCmd last_cmd_;
  // unitree_msgs::msg::MotorState last_state_;
  ServoCmd servo_cmd_;  // use for `computeTorque`

  // break
  bool is_halted_ = false;
  bool use_stamped_vel_ = true;
  bool reset();
  void halt();

  /**
   * @brief 这里的值是直接从硬件中 read()/write() 通过进程内通信(share memory)获得
   * 还可以再封装一层！直接对每个interface的 ->set()/->get()
   */
  struct JointHandle {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback_position;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback_velocity;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> command_velocity;
  };

  /**
   * <joint name="${prefix}joint_ll_0_name">
   *   <command_interface name="velocity"/>
   *   <state_interface name="position"/>
   *   <state_interface name="velocity"/>
   *   <param name="id">1</param>
   * </joint>
   */
  std::vector<JointHandle> registered_joint_handles_;

  /**
   * @brief find joint interface handle from `state_interfaces_` and `command_interfaces_`
   * @details find all joint interface include state, command from controller_manger(controller_interface) and
   * resource_manager(hardware_interface)
   * @param joints_name
   * @param registered_handles
   * @return controller_interface::CallbackReturn
   */
  controller_interface::CallbackReturn get_joint_handle(
      const std::vector<std::string> &joints_name,  //
      std::vector<JointHandle> &registered_handles);
};
}  // namespace unitree_joint_controller

#endif  // UNITREE_JOINT_CONTROLLER__UNITREE_JOINT_CONTROLLER_HPP_
