/**
 * @brief Collect messages from gazebo and transpond to visualization module.
 */

#ifndef NOT_USE_SIM
#ifndef QR_CONTROL2GAZEBO_MSG_H_
#define QR_CONTROL2GAZEBO_MSG_H_

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"

#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// #include "xpp_msgs/msg/robot_parameters.hpp"
// #include "xpp_msgs/msg/robot_state_cartesian.hpp"
// #include "xpp_msgs/msg/terrain_info.hpp"
// #include "xpp_states/convert.h"
// #include "xpp_states/robot_state_cartesian.h"

#include "quadruped/controllers/qr_locomotion_controller.h"

namespace Quadruped {

class qrController2GazeboMsg : public rclcpp::Node {
public:
  qrController2GazeboMsg(qrRobot *robotIn, qrLocomotionController *locomotionController);

  void PublishGazeboStateCallback();

private:
  // rclcpp::Publisher<xpp_msgs::msg::RobotStateCartesian>::SharedPtr gazeboStatePublish;
  // rclcpp::Publisher<xpp_msgs::msg::RobotParameters>::SharedPtr gazeboParamPublish;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr posePlannerPublish;
  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr baseStateClient;
  std::shared_ptr<tf2_ros::TransformBroadcaster> desiredPoseFramebr;

  qrRobot *robot;
  qrLocomotionController *locomotionController;
  qrPosePlanner *posePlanner;

  Vec3<float> rIB;
  Quat<float> quat;
  geometry_msgs::msg::Transform transform;
  rclcpp::Time currentTime;
  rclcpp::Time lastTime;
};

}  // namespace Quadruped

#endif  // QR_CONTROL2GAZEBO_MSG_H_
#endif
