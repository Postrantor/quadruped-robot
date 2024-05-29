/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef NOT_USE_SIM
#ifndef QR_CONTROL2GAZEBO_MSG_H_
#define QR_CONTROL2GAZEBO_MSG_H_

#include <gazebo_msgs/GetLinkState.h>
#include <geometry_msgs/Pose.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include "controllers/qr_locomotion_controller.h"
#include "xpp_msgs/RobotParameters.h"
#include "xpp_msgs/RobotStateCartesian.h"
#include "xpp_msgs/TerrainInfo.h"
#include "xpp_msgs/topic_names.h"
#include "xpp_states/convert.h"
#include "xpp_states/robot_state_cartesian.h"

namespace Quadruped {

/**
 * @brief Collect messages from gazebo and transpond to visualization module.
 */
class qrController2GazeboMsg {
public:
  qrController2GazeboMsg(qrRobot *robotIn, qrLocomotionController *locomotionController, ros::NodeHandle &nhIn);

  void PublishGazeboStateCallback();

private:
  qrRobot *robot;
  qrLocomotionController *locomotionController;
  qrPosePlanner *posePlanner;
  ros::NodeHandle &nh;
  ros::Publisher gazeboStatePublish;
  ros::Publisher gazeboParamPublish;
  ros::Publisher posePlannerPublish;
  ros::ServiceClient baseStateClient;
  tf::TransformBroadcaster desiredPoseFramebr;
  tf::Transform transform;
  Vec3<float> rIB;
  Quat<float> quat;
  ros::Time currentTime;
  ros::Time lastTime;
};

}  // Namespace Quadruped

#endif  // QR_CONTROL2GAZEBO_MSG_H_
#endif
