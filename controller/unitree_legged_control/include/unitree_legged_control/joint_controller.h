/**
 * @author GPT4-o
 * @brief Joint controller for Unitree Legged Robots
 * @date 2024-06-23 16:55:13
 */

#ifndef UNITREE_JOINT_CONTROLLER_HPP_
#define UNITREE_JOINT_CONTROLLER_HPP_

#include "unitree_legged_control/unitree_joint_control_tool.hpp"

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

#include "unitree_msgs/msg/motor_cmd.hpp"
#include "unitree_msgs/msg/motor_state.hpp"

namespace unitree_legged_control {

class UnitreeJointController : public controller_interface::ControllerInterface {
public:
  UnitreeJointController();
  ~UnitreeJointController() override;

  controller_interface::return_type init(const std::string &controller_name) override;
  controller_interface::return_type update() override;
  controller_interface::return_type on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::return_type on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

private:
  hardware_interface::LoanedCommandInterface joint_;

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_ft_;
  rclcpp::Subscription<unitree_msgs::msg::MotorCmd>::SharedPtr sub_cmd_;
  rclcpp::Publisher<unitree_msgs::msg::MotorState>::SharedPtr pub_state_;

  realtime_tools::RealtimeBuffer<unitree_msgs::msg::MotorCmd> command_buffer_;
  unitree_msgs::msg::MotorCmd last_cmd_;
  unitree_msgs::msg::MotorState last_state_;
  ServoCmd servo_cmd_;

  void setTorqueCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void setCommandCB(const unitree_msgs::msg::MotorCmd::SharedPtr msg);
  void positionLimits(double &position);
  void velocityLimits(double &velocity);
  void effortLimits(double &effort);

  void setGains(
      const double &p,
      const double &i,
      const double &d,
      const double &i_max,
      const double &i_min,
      const bool &antiwindup = false);
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
};

}  // namespace unitree_legged_control

#endif  // UNITREE_JOINT_CONTROLLER_HPP_
