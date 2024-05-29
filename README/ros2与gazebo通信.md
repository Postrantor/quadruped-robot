# plugin

## **You:**

这是 ros2 与 gazebo 相关的代码，请帮我梳理其中的逻辑，详细阐述 ros2 与 gazebo 相互交互的过程

```cpp
/*
 * \file  gazebo_ros_diff_drive.cpp
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */

/*
 *
 * The support of acceleration limit was added by
 * \author   George Todoran <todorangrg@gmail.com>
 * \author   Markus Bader <markus.bader@tuwien.ac.at>
 * \date 22th of May 2014
 */

#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_diff_drive.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdf/sdf.hh>

#ifdef NO_ERROR
// NO_ERROR is a macro defined in Windows that's used as an enum in tf2
#undef NO_ERROR
#endif

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins {
class GazeboRosDiffDrivePrivate {
public:
  /// Indicates where the odometry info is coming from
  enum OdomSource {
    /// Use an ancoder
    ENCODER = 0,

    /// Use ground truth from simulation world
    WORLD = 1,
  };

  /// Indicates which wheel
  enum {
    /// Right wheel
    RIGHT = 0,

    /// Left wheel
    LEFT = 1,
  };

  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo& _info);

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

  /// Update wheel velocities according to latest target velocities.
  void UpdateWheelVelocities();

  /// Update odometry according to encoder.
  /// \param[in] _current_time Current simulation time
  void UpdateOdometryEncoder(const gazebo::common::Time& _current_time);

  /// Update odometry according to world
  void UpdateOdometryWorld();

  /// Publish odometry transforms
  /// \param[in] _current_time Current simulation time
  void PublishOdometryTf(const gazebo::common::Time& _current_time);

  /// Publish trasforms for the wheels
  /// \param[in] _current_time Current simulation time
  void PublishWheelsTf(const gazebo::common::Time& _current_time);

  /// Publish odometry messages
  /// \param[in] _current_time Current simulation time
  void PublishOdometryMsg(const gazebo::common::Time& _current_time);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Distance between the wheels, in meters.
  std::vector<double> wheel_separation_;

  /// Diameter of wheels, in meters.
  std::vector<double> wheel_diameter_;

  /// Maximum wheel torque, in Nm.
  double max_wheel_torque_;

  /// Maximum wheel acceleration
  double max_wheel_accel_;

  /// Desired wheel speed.
  std::vector<double> desired_wheel_speed_;

  /// Speed sent to wheel.
  std::vector<double> wheel_speed_instr_;

  /// Pointers to wheel joints.
  std::vector<gazebo::physics::JointPtr> joints_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// To broadcast TFs
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Linear velocity in X received on command (m/s).
  double target_x_{0.0};

  /// Angular velocity in Z received on command (rad/s).
  double target_rot_{0.0};

  /// Update period in seconds.
  double update_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// Keep encoder data.
  geometry_msgs::msg::Pose2D pose_encoder_;

  /// Odometry frame ID
  std::string odometry_frame_;

  /// Last time the encoder was updated
  gazebo::common::Time last_encoder_update_;

  /// Either ENCODER or WORLD
  OdomSource odom_source_;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom_;

  /// Robot base frame ID
  std::string robot_base_frame_;

  /// True to publish odometry messages.
  bool publish_odom_;

  /// True to publish wheel-to-base transforms.
  bool publish_wheel_tf_;

  /// True to publish odom-to-world transforms.
  bool publish_odom_tf_;

  /// Store number of wheel pairs
  unsigned int num_wheel_pairs_;

  /// Covariance in odometry
  double covariance_[3];
};

GazeboRosDiffDrive::GazeboRosDiffDrive() : impl_(std::make_unique<GazeboRosDiffDrivePrivate>()) {}

GazeboRosDiffDrive::~GazeboRosDiffDrive() {}

void GazeboRosDiffDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  impl_->model_ = _model;

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS& qos = impl_->ros_node_->get_qos();

  // Get number of wheel pairs in the model
  impl_->num_wheel_pairs_ = static_cast<unsigned int>(_sdf->Get<int>("num_wheel_pairs", 1).first);

  if (impl_->num_wheel_pairs_ < 1) {
    impl_->num_wheel_pairs_ = 1;
    RCLCPP_WARN(
        impl_->ros_node_->get_logger(), "Drive requires at least one pair of wheels. Setting [num_wheel_pairs] to 1");
  }

  // Dynamic properties
  impl_->max_wheel_accel_ = _sdf->Get<double>("max_wheel_acceleration", 0.0).first;
  impl_->max_wheel_torque_ = _sdf->Get<double>("max_wheel_torque", 5.0).first;

  // Get joints and Kinematic properties
  std::vector<gazebo::physics::JointPtr> left_joints, right_joints;

  for (auto left_joint_elem = _sdf->GetElement("left_joint"); left_joint_elem != nullptr;
       left_joint_elem = left_joint_elem->GetNextElement("left_joint")) {
    auto left_joint_name = left_joint_elem->Get<std::string>();
    auto left_joint = _model->GetJoint(left_joint_name);
    if (!left_joint) {
      RCLCPP_ERROR(
          impl_->ros_node_->get_logger(), "Joint [%s] not found, plugin will not work.", left_joint_name.c_str());
      impl_->ros_node_.reset();
      return;
    }
    left_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
    left_joints.push_back(left_joint);
  }

  for (auto right_joint_elem = _sdf->GetElement("right_joint"); right_joint_elem != nullptr;
       right_joint_elem = right_joint_elem->GetNextElement("right_joint")) {
    auto right_joint_name = right_joint_elem->Get<std::string>();
    auto right_joint = _model->GetJoint(right_joint_name);
    if (!right_joint) {
      RCLCPP_ERROR(
          impl_->ros_node_->get_logger(), "Joint [%s] not found, plugin will not work.", right_joint_name.c_str());
      impl_->ros_node_.reset();
      return;
    }
    right_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
    right_joints.push_back(right_joint);
  }

  if (left_joints.size() != right_joints.size() || left_joints.size() != impl_->num_wheel_pairs_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Inconsistent number of joints specified. Plugin will not work.");
    impl_->ros_node_.reset();
    return;
  }

  unsigned int index;
  for (index = 0; index < impl_->num_wheel_pairs_; ++index) {
    impl_->joints_.push_back(right_joints[index]);
    impl_->joints_.push_back(left_joints[index]);
  }

  index = 0;
  impl_->wheel_separation_.assign(impl_->num_wheel_pairs_, 0.34);
  for (auto wheel_separation = _sdf->GetElement("wheel_separation"); wheel_separation != nullptr;
       wheel_separation = wheel_separation->GetNextElement("wheel_separation")) {
    if (index >= impl_->num_wheel_pairs_) {
      RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <wheel_separation>");
      break;
    }
    impl_->wheel_separation_[index] = wheel_separation->Get<double>();
    RCLCPP_INFO(
        impl_->ros_node_->get_logger(), "Wheel pair %i separation set to [%fm]", index + 1,
        impl_->wheel_separation_[index]);
    index++;
  }

  index = 0;
  impl_->wheel_diameter_.assign(impl_->num_wheel_pairs_, 0.15);
  for (auto wheel_diameter = _sdf->GetElement("wheel_diameter"); wheel_diameter != nullptr;
       wheel_diameter = wheel_diameter->GetNextElement("wheel_diameter")) {
    if (index >= impl_->num_wheel_pairs_) {
      RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <wheel_diameter>");
      break;
    }
    impl_->wheel_diameter_[index] = wheel_diameter->Get<double>();
    RCLCPP_INFO(
        impl_->ros_node_->get_logger(), "Wheel pair %i diameter set to [%fm]", index + 1,
        impl_->wheel_diameter_[index]);
    index++;
  }

  impl_->wheel_speed_instr_.assign(2 * impl_->num_wheel_pairs_, 0);
  impl_->desired_wheel_speed_.assign(2 * impl_->num_wheel_pairs_, 0);

  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
      std::bind(&GazeboRosDiffDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->cmd_vel_sub_->get_topic_name());

  // Odometry
  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
  impl_->odom_source_ = static_cast<GazeboRosDiffDrivePrivate::OdomSource>(_sdf->Get<int>("odometry_source", 1).first);

  // Advertise odometry topic
  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", false).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
        "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Advertise odometry on [%s]", impl_->odometry_pub_->get_topic_name());
  }

  // Create TF broadcaster if needed
  impl_->publish_wheel_tf_ = _sdf->Get<bool>("publish_wheel_tf", false).first;
  impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", false).first;
  if (impl_->publish_wheel_tf_ || impl_->publish_odom_tf_) {
    impl_->transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

    if (impl_->publish_odom_tf_) {
      RCLCPP_INFO(
          impl_->ros_node_->get_logger(), "Publishing odom transforms between [%s] and [%s]",
          impl_->odometry_frame_.c_str(), impl_->robot_base_frame_.c_str());
    }

    for (index = 0; index < impl_->num_wheel_pairs_; ++index) {
      if (impl_->publish_wheel_tf_) {
        RCLCPP_INFO(
            impl_->ros_node_->get_logger(), "Publishing wheel transforms between [%s], [%s] and [%s]",
            impl_->robot_base_frame_.c_str(),
            impl_->joints_[2 * index + GazeboRosDiffDrivePrivate::LEFT]->GetName().c_str(),
            impl_->joints_[2 * index + GazeboRosDiffDrivePrivate::RIGHT]->GetName().c_str());
      }
    }
  }

  impl_->covariance_[0] = _sdf->Get<double>("covariance_x", 0.00001).first;
  impl_->covariance_[1] = _sdf->Get<double>("covariance_y", 0.00001).first;
  impl_->covariance_[2] = _sdf->Get<double>("covariance_yaw", 0.001).first;

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosDiffDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosDiffDrive::Reset() {
  impl_->last_update_time_ = impl_->joints_[GazeboRosDiffDrivePrivate::LEFT]->GetWorld()->SimTime();
  for (unsigned int i = 0; i < impl_->num_wheel_pairs_; ++i) {
    if (impl_->joints_[2 * i + GazeboRosDiffDrivePrivate::LEFT] &&
        impl_->joints_[2 * i + GazeboRosDiffDrivePrivate::RIGHT]) {
      impl_->joints_[2 * i + GazeboRosDiffDrivePrivate::LEFT]->SetParam("fmax", 0, impl_->max_wheel_torque_);
      impl_->joints_[2 * i + GazeboRosDiffDrivePrivate::RIGHT]->SetParam("fmax", 0, impl_->max_wheel_torque_);
    }
  }
  impl_->pose_encoder_.x = 0;
  impl_->pose_encoder_.y = 0;
  impl_->pose_encoder_.theta = 0;
  impl_->target_x_ = 0;
  impl_->target_rot_ = 0;
}

void GazeboRosDiffDrivePrivate::OnUpdate(const gazebo::common::UpdateInfo& _info) {
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosDiffDrivePrivate::OnUpdate");
#endif
  // Update encoder even if we're going to skip this update
  if (odom_source_ == ENCODER) {
    UpdateOdometryEncoder(_info.simTime);
  }

  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("UpdateOdometryWorld");
#endif
  // Update odom message if using ground truth
  if (odom_source_ == WORLD) {
    UpdateOdometryWorld();
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("PublishOdometryMsg");
#endif
  if (publish_odom_) {
    PublishOdometryMsg(_info.simTime);
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("PublishWheelsTf");
#endif
  if (publish_wheel_tf_) {
    PublishWheelsTf(_info.simTime);
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("PublishOdometryTf");
#endif
  if (publish_odom_tf_) {
    PublishOdometryTf(_info.simTime);
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("UpdateWheelVelocities");
#endif
  // Update robot in case new velocities have been requested
  UpdateWheelVelocities();
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  // Current speed
  std::vector<double> current_speed(2 * num_wheel_pairs_);
  for (unsigned int i = 0; i < num_wheel_pairs_; ++i) {
    current_speed[2 * i + LEFT] = joints_[2 * i + LEFT]->GetVelocity(0) * (wheel_diameter_[i] / 2.0);
    current_speed[2 * i + RIGHT] = joints_[2 * i + RIGHT]->GetVelocity(0) * (wheel_diameter_[i] / 2.0);
  }

  // If max_accel == 0, or target speed is reached
  for (unsigned int i = 0; i < num_wheel_pairs_; ++i) {
    if (max_wheel_accel_ == 0 || ((fabs(desired_wheel_speed_[2 * i + LEFT] - current_speed[2 * i + LEFT]) < 0.01) &&
                                  (fabs(desired_wheel_speed_[2 * i + RIGHT] - current_speed[2 * i + RIGHT]) < 0.01))) {
      joints_[2 * i + LEFT]->SetParam("vel", 0, desired_wheel_speed_[2 * i + LEFT] / (wheel_diameter_[i] / 2.0));
      joints_[2 * i + RIGHT]->SetParam("vel", 0, desired_wheel_speed_[2 * i + RIGHT] / (wheel_diameter_[i] / 2.0));
    } else {
      if (desired_wheel_speed_[2 * i + LEFT] >= current_speed[2 * i + LEFT]) {
        wheel_speed_instr_[2 * i + LEFT] += fmin(
            desired_wheel_speed_[2 * i + LEFT] - current_speed[2 * i + LEFT],
            max_wheel_accel_ * seconds_since_last_update);
      } else {
        wheel_speed_instr_[2 * i + LEFT] += fmax(
            desired_wheel_speed_[2 * i + LEFT] - current_speed[2 * i + LEFT],
            -max_wheel_accel_ * seconds_since_last_update);
      }

      if (desired_wheel_speed_[2 * i + RIGHT] > current_speed[2 * i + RIGHT]) {
        wheel_speed_instr_[2 * i + RIGHT] += fmin(
            desired_wheel_speed_[2 * i + RIGHT] - current_speed[2 * i + RIGHT],
            max_wheel_accel_ * seconds_since_last_update);
      } else {
        wheel_speed_instr_[2 * i + RIGHT] += fmax(
            desired_wheel_speed_[2 * i + RIGHT] - current_speed[2 * i + RIGHT],
            -max_wheel_accel_ * seconds_since_last_update);
      }

      joints_[2 * i + LEFT]->SetParam("vel", 0, wheel_speed_instr_[2 * i + LEFT] / (wheel_diameter_[i] / 2.0));
      joints_[2 * i + RIGHT]->SetParam("vel", 0, wheel_speed_instr_[2 * i + RIGHT] / (wheel_diameter_[i] / 2.0));
    }
  }

  last_update_time_ = _info.simTime;
}

void GazeboRosDiffDrivePrivate::UpdateWheelVelocities() {
  std::lock_guard<std::mutex> scoped_lock(lock_);

  double vr = target_x_;
  double va = target_rot_;

  for (unsigned int i = 0; i < num_wheel_pairs_; ++i) {
    desired_wheel_speed_[2 * i + LEFT] = vr - va * wheel_separation_[i] / 2.0;
    desired_wheel_speed_[2 * i + RIGHT] = vr + va * wheel_separation_[i] / 2.0;
  }
}

void GazeboRosDiffDrivePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg) {
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_x_ = _msg->linear.x;
  target_rot_ = _msg->angular.z;
}

void GazeboRosDiffDrivePrivate::UpdateOdometryEncoder(const gazebo::common::Time& _current_time) {
  double vl = joints_[LEFT]->GetVelocity(0);
  double vr = joints_[RIGHT]->GetVelocity(0);

  double seconds_since_last_update = (_current_time - last_encoder_update_).Double();
  last_encoder_update_ = _current_time;

  double b = wheel_separation_[0];

  // Book: Sigwart 2011 Autonompus Mobile Robots page:337
  double sl = vl * (wheel_diameter_[0] / 2.0) * seconds_since_last_update;
  double sr = vr * (wheel_diameter_[0] / 2.0) * seconds_since_last_update;
  double ssum = sl + sr;

  double sdiff = sr - sl;

  double dx = (ssum) / 2.0 * cos(pose_encoder_.theta + (sdiff) / (2.0 * b));
  double dy = (ssum) / 2.0 * sin(pose_encoder_.theta + (sdiff) / (2.0 * b));
  double dtheta = (sdiff) / b;

  pose_encoder_.x += dx;
  pose_encoder_.y += dy;
  pose_encoder_.theta += dtheta;

  double w = dtheta / seconds_since_last_update;
  double v = sqrt(dx * dx + dy * dy) / seconds_since_last_update;

  tf2::Quaternion qt;
  tf2::Vector3 vt;
  qt.setRPY(0, 0, pose_encoder_.theta);
  vt = tf2::Vector3(pose_encoder_.x, pose_encoder_.y, 0);

  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z();

  odom_.pose.pose.orientation.x = qt.x();
  odom_.pose.pose.orientation.y = qt.y();
  odom_.pose.pose.orientation.z = qt.z();
  odom_.pose.pose.orientation.w = qt.w();

  odom_.twist.twist.angular.z = w;
  odom_.twist.twist.linear.x = v;
  odom_.twist.twist.linear.y = 0;
}

void GazeboRosDiffDrivePrivate::UpdateOdometryWorld() {
  auto pose = model_->WorldPose();
  odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  // Get velocity in odom frame
  auto linear = model_->WorldLinearVel();
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  // Convert velocity to child_frame_id(aka base_footprint)
  float yaw = pose.Rot().Yaw();
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
}

void GazeboRosDiffDrivePrivate::PublishOdometryTf(const gazebo::common::Time& _current_time) {
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
  msg.transform.rotation = odom_.pose.pose.orientation;

  transform_broadcaster_->sendTransform(msg);
}

void GazeboRosDiffDrivePrivate::PublishWheelsTf(const gazebo::common::Time& _current_time) {
  for (unsigned int i = 0; i < 2 * num_wheel_pairs_; ++i) {
    auto pose_wheel = joints_[i]->GetChild()->RelativePose();

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
    msg.header.frame_id = joints_[i]->GetParent()->GetName();
    msg.child_frame_id = joints_[i]->GetChild()->GetName();
    msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose_wheel.Pos());
    msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose_wheel.Rot());

    transform_broadcaster_->sendTransform(msg);
  }
}

void GazeboRosDiffDrivePrivate::PublishOdometryMsg(const gazebo::common::Time& _current_time) {
  // Set covariance
  odom_.pose.covariance[0] = covariance_[0];
  odom_.pose.covariance[7] = covariance_[1];
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = covariance_[2];

  odom_.twist.covariance[0] = covariance_[0];
  odom_.twist.covariance[7] = covariance_[1];
  odom_.twist.covariance[14] = 1000000000000.0;
  odom_.twist.covariance[21] = 1000000000000.0;
  odom_.twist.covariance[28] = 1000000000000.0;
  odom_.twist.covariance[35] = covariance_[2];

  // Set header
  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  // Publish
  odometry_pub_->publish(odom_);
}
GZ_REGISTER_MODEL_PLUGIN(GazeboRosDiffDrive)
}  // namespace gazebo_plugins
```

---

## **ChatGPT:**

这段代码实现了一个差速驱动(differential drive)插件，用于在 Gazebo 模拟器中控制机器人模型，并与 ROS 2 进行通信。以下是其主要功能和逻辑的详细解析：

### 1. 插件的初始化与加载

#### `GazeboRosDiffDrive::Load()`

- **模型与节点初始化**：
  - 插件首先通过参数获取与模型相关的信息，如轮子的数量、最大轮子加速度、最大轮子扭矩等。
  - 初始化 ROS 节点，通过`gazebo_ros::Node::Get`方法获取一个共享指针，用于与 ROS 2 进行通信。
- **获取关节和运动学属性**：
  - 通过解析 SDF(Simulation Description Format)文件获取左右轮子的关节名称，并初始化这些关节。
  - 为这些关节设置最大扭矩参数。
- **订阅速度命令**：
  - 通过 ROS 2 订阅`geometry_msgs::msg::Twist`类型的主题(通常命名为`cmd_vel`)，以接收速度命令。
- **设置里程计**：
  - 配置里程计相关参数，如里程计帧 ID、机器人基础帧 ID、里程计源(编码器或世界坐标系)、协方差等。
  - 如果需要发布里程计信息，初始化相应的 ROS 2 发布者。
- **设置 TF 广播**：
  - 如果需要发布 TF 变换，初始化 TF 广播器，用于广播机器人各部分之间的变换。
- **连接 Gazebo 事件**：
  - 通过`ConnectWorldUpdateBegin`方法连接 Gazebo 的世界更新事件，在每次世界更新时调用`OnUpdate`方法。

### 2. 主要回调函数

#### `GazeboRosDiffDrivePrivate::OnUpdate()`

- **更新里程计**：
  - 根据里程计源(编码器或世界坐标系)更新里程计数据。
  - 如果需要，发布里程计消息和 TF 变换。
- **更新轮子速度**：
  - 根据接收到的目标速度，更新实际发送到轮子的速度指令。
  - 计算每个轮子的当前速度，并将其调整到目标速度。

#### `GazeboRosDiffDrivePrivate::OnCmdVel()`

- **处理速度命令**：
  - 接收并处理来自 ROS 2 的速度命令，将其保存为目标线速度和角速度。

### 3. 里程计更新

#### `GazeboRosDiffDrivePrivate::UpdateOdometryEncoder()`

- **基于编码器的里程计更新**：
  - 计算每个轮子的线速度，根据时间差计算机器人在平面上的移动距离和角度变化。
  - 更新机器人的位置和方向，计算并存储当前速度和角速度。

#### `GazeboRosDiffDrivePrivate::UpdateOdometryWorld()`

- **基于世界坐标系的里程计更新**：
  - 获取模型在世界坐标系中的位置和方向，计算线速度和角速度。
  - 将这些信息转换为里程计消息的格式。

### 4. TF 发布

#### `GazeboRosDiffDrivePrivate::PublishOdometryTf()`

- **发布里程计 TF 变换**：
  - 将里程计信息发布为 TF 变换，用于其他 ROS 2 节点获取机器人的位置和方向。

#### `GazeboRosDiffDrivePrivate::PublishWheelsTf()`

- **发布轮子 TF 变换**：
  - 将每个轮子的变换发布为 TF 变换，提供轮子相对于机器人基座的位置信息。

### 5. 速度控制

#### `GazeboRosDiffDrivePrivate::UpdateWheelVelocities()`

- **计算目标速度**：
  - 根据接收到的目标线速度和角速度，计算每个轮子的目标速度。
  - 调整实际发送到轮子的速度，使其逐步达到目标速度。

### 总结

这段代码实现了一个差速驱动插件，能够在 Gazebo 中模拟机器人运动，并通过 ROS 2 与外部进行通信。它实现了从接收速度命令、计算和更新轮子速度、里程计数据的计算与发布、到 TF 变换的发布等一系列功能，保证机器人在模拟环境中能够按照预期的速度和轨迹运动。

---

## **You:**

以这段代码为例，详细介绍一下 ros2 与 gazebo 仿真机器人的交互逻辑

---

## **ChatGPT:**

这段代码展示了一个在 ROS 2 和 Gazebo 中用于差速驱动机器人的插件 `GazeboRosDiffDrive`。插件主要实现了机器人接收速度命令、计算轮速和里程计数据、并发布相应的 ROS 2 消息和坐标变换。以下是 ROS 2 和 Gazebo 如何交互的详细介绍：

### 1. 插件加载和初始化

当插件被加载时，会执行 `Load` 函数，该函数的主要任务包括：

- 获取模型指针和 SDF 元素。
- 初始化 ROS 2 节点。
- 获取和设置机器人差速驱动的各种参数，如轮子的数量、轮子之间的距离、轮子的直径、最大轮子扭矩等。
- 创建 ROS 2 订阅者来接收速度命令(`cmd_vel`)。
- 创建 ROS 2 发布者来发布里程计数据(`odom`)。
- 设置 Gazebo 更新事件的回调函数，用于在每次仿真更新时调用。

```cpp
void GazeboRosDiffDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    // 初始化模型和ROS节点
    impl_->model_ = _model;
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

    // 设置速度命令订阅者
    impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
        std::bind(&GazeboRosDiffDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

    // 设置里程计发布者
    impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", false).first;
    if (impl_->publish_odom_) {
        impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
            "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));
    }

    // 设置TF广播器
    impl_->publish_wheel_tf_ = _sdf->Get<bool>("publish_wheel_tf", false).first;
    impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", false).first;
    if (impl_->publish_wheel_tf_ || impl_->publish_odom_tf_) {
        impl_->transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);
    }

    // 连接到世界更新事件
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosDiffDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}
```

### 2. 接收速度命令

当 ROS 2 收到速度命令(`cmd_vel`)时，会触发 `OnCmdVel` 回调函数。该函数将速度命令保存为目标线速度和角速度。

```cpp
void GazeboRosDiffDrivePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg) {
    std::lock_guard<std::mutex> scoped_lock(lock_);
    target_x_ = _msg->linear.x;
    target_rot_ = _msg->angular.z;
}
```

### 3. 仿真更新

每次仿真更新时，Gazebo 会调用 `OnUpdate` 函数。这个函数完成以下几项任务：

- 更新编码器的里程计数据或根据仿真世界的真实数据更新里程计数据。
- 根据需要发布里程计消息和坐标变换。
- 更新轮子的目标速度。
- 计算并设置轮子的当前速度。

```cpp
void GazeboRosDiffDrivePrivate::OnUpdate(const gazebo::common::UpdateInfo& _info) {
    // 更新编码器里程计数据
    if (odom_source_ == ENCODER) {
        UpdateOdometryEncoder(_info.simTime);
    }

    double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
    if (seconds_since_last_update < update_period_) {
        return;
    }

    // 更新世界真实里程计数据
    if (odom_source_ == WORLD) {
        UpdateOdometryWorld();
    }

    // 发布里程计消息
    if (publish_odom_) {
        PublishOdometryMsg(_info.simTime);
    }

    // 发布轮子的TF变换
    if (publish_wheel_tf_) {
        PublishWheelsTf(_info.simTime);
    }

    // 发布里程计TF变换
    if (publish_odom_tf_) {
        PublishOdometryTf(_info.simTime);
    }

    // 更新轮子速度
    UpdateWheelVelocities();
    last_update_time_ = _info.simTime;
}
```

### 4. 计算和发布里程计数据

`UpdateOdometryEncoder` 和 `UpdateOdometryWorld` 函数分别用于根据编码器数据或仿真世界的真实数据计算里程计信息。计算结果通过 `PublishOdometryMsg` 和 `PublishOdometryTf` 函数发布。

```cpp
void GazeboRosDiffDrivePrivate::UpdateOdometryEncoder(const gazebo::common::Time& _current_time) {
    double vl = joints_[LEFT]->GetVelocity(0);
    double vr = joints_[RIGHT]->GetVelocity(0);
    double seconds_since_last_update = (_current_time - last_encoder_update_).Double();
    last_encoder_update_ = _current_time;

    double b = wheel_separation_[0];
    double sl = vl * (wheel_diameter_[0] / 2.0) * seconds_since_last_update;
    double sr = vr * (wheel_diameter_[0] / 2.0) * seconds_since_last_update;
    double ssum = sl + sr;
    double sdiff = sr - sl;
    double dx = (ssum) / 2.0 * cos(pose_encoder_.theta + (sdiff) / (2.0 * b));
    double dy = (ssum) / 2.0 * sin(pose_encoder_.theta + (sdiff) / (2.0 * b));
    double dtheta = (sdiff) / b;

    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;

    odom_.pose.pose.position.x = pose_encoder_.x;
    odom_.pose.pose.position.y = pose_encoder_.y;
    odom_.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, pose_encoder_.theta));

    odom_.twist.twist.linear.x = dx / seconds_since_last_update;
    odom_.twist.twist.angular.z = dtheta / seconds_since_last_update;
}

void GazeboRosDiffDrivePrivate::PublishOdometryMsg(const gazebo::common::Time& _current_time) {
    odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
    odom_.header.frame_id = odometry_frame_;
    odom_.child_frame_id = robot_base_frame_;
    odometry_pub_->publish(odom_);
}

void GazeboRosDiffDrivePrivate::PublishOdometryTf(const gazebo::common::Time& _current_time) {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
    msg.header.frame_id = odometry_frame_;
    msg.child_frame_id = robot_base_frame_;
    msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
    msg.transform.rotation = odom_.pose.pose.orientation;
    transform_broadcaster_->sendTransform(msg);
}
```

### 5. 更新轮子速度

在 `UpdateWheelVelocities` 函数中，根据目标速度计算每个轮子的目标速度，并在 `OnUpdate` 函数中设置这些速度。

```cpp
void GazeboRosDiffDrivePrivate::UpdateWheelVelocities() {
    std::lock_guard<std::mutex> scoped_lock(lock_);

    double vr = target_x_;
    double va = target_rot_;

    for (unsigned int i = 0; i < num_wheel_pairs_; ++i) {
        desired_wheel_speed_[2 * i + LEFT] = vr - va * wheel_separation_[i] / 2.0;
        desired_wheel_speed_[2 * i + RIGHT] = vr + va * wheel_separation_[i] / 2.0;
    }
}
```

### 总结

- 插件通过 ROS 2 接收速度命令，并将其转换为每个轮子的目标速度。
- 插件在每次仿真更新时计算和发布里程计数据。
- 插件通过 ROS 2 发布里程计消息和坐标变换，确保机器人状态在 ROS 2 中是最新的。
- 插件利用 Gazebo 提供的物理仿真环境计算实际的轮子速度和位置变化，并根据这些数据更新机器人状态。

---

## **You:**

这段代码是对应的 hpp 文件，请从各个类之间的关系出发，再梳理一下其中的逻辑关系：

```cpp
#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_DIFF_DRIVE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_DIFF_DRIVE_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins {
class GazeboRosDiffDrivePrivate;

/// A differential drive plugin for gazebo. Based on the diffdrive plugin
/*
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */
/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_diff_drive" filename="libgazebo_ros_diff_drive.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/test</namespace>

      </ros>

      <!-- Update rate in Hz -->
      <update_rate>50</update_rate>

      <!-- wheels -->
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>

      <!-- kinematics -->
      <wheel_separation>1.25</wheel_separation>
      <wheel_diameter>0.6</wheel_diameter>

      <!-- limits -->
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>

      <!-- input -->
      <command_topic>cmd_vel</command_topic>

      <!-- output -->
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>

      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>chassis</robot_base_frame>

    </plugin>
  \endcode
*/
class GazeboRosDiffDrive : public gazebo::ModelPlugin {
public:
  /// Constructor
  GazeboRosDiffDrive();

  /// Destructor
  ~GazeboRosDiffDrive();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosDiffDrivePrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_DIFF_DRIVE_HPP_
```

---

## **ChatGPT:**

这段代码定义了一个差速驱动插件 `GazeboRosDiffDrive`，用于在 Gazebo 仿真环境中控制机器人。它与 ROS 2 集成，使得机器人可以通过 ROS 2 消息接收速度命令并发布里程计数据。以下是类与类之间的逻辑关系和实现的详细梳理：

### 类和命名空间的概述

#### `gazebo_plugins` 命名空间

所有插件的实现都在 `gazebo_plugins` 命名空间中，确保代码的模块化和避免命名冲突。

#### `GazeboRosDiffDrive` 类

`GazeboRosDiffDrive` 是一个公有继承自 `gazebo::ModelPlugin` 的类，代表了一个 Gazebo 模型插件。

- **构造函数和析构函数**
  - `GazeboRosDiffDrive()`：构造函数，用于初始化插件。
  - `~GazeboRosDiffDrive()`：析构函数，用于清理资源。
- **保护成员函数**
  - `void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override`：插件加载函数，在插件被插入仿真环境时调用，用于初始化插件的各种参数和 ROS 2 节点。
  - `void Reset() override`：重置函数，用于在仿真重置时恢复插件的初始状态。
- **私有成员**
  - `std::unique_ptr<GazeboRosDiffDrivePrivate> impl_`：指向 `GazeboRosDiffDrivePrivate` 类的指针，用于实现插件的具体逻辑，保持接口的简洁。

#### `GazeboRosDiffDrivePrivate` 类

这个类通常定义在 `.cpp` 文件中，包含了插件的具体实现。它主要包括：

- ROS 2 节点的初始化和处理。
- Gazebo 仿真环境的交互，包括物理参数的设置、速度命令的处理和里程计数据的发布。
- 事件回调函数的定义和实现，如速度命令的接收和仿真更新处理。

### 类之间的关系

#### `GazeboRosDiffDrive` 和 `gazebo::ModelPlugin`

`GazeboRosDiffDrive` 类继承自 `gazebo::ModelPlugin`，意味着它是一个 Gazebo 模型插件。`gazebo::ModelPlugin` 提供了插件在仿真环境中所需的基本接口，如 `Load` 和 `Reset` 函数。通过重载这些函数，`GazeboRosDiffDrive` 实现了自定义的加载和重置行为。

#### `GazeboRosDiffDrive` 和 `GazeboRosDiffDrivePrivate`

`GazeboRosDiffDrive` 类使用 `GazeboRosDiffDrivePrivate` 类来封装具体的实现细节。这种设计模式称为**Pimpl(Pointer to Implementation)**，它有以下几个好处：

- 隐藏实现细节，使得接口更加简洁。
- 提高编译速度，因为更改实现细节时不需要重新编译接口头文件。
- 提供更好的封装性和数据隐藏。

### 插件的工作流程

1. **插件加载和初始化**

   - 当插件被插入到仿真模型中时，Gazebo 调用 `Load` 函数。
   - `Load` 函数通过 `_model` 和 `_sdf` 参数获取模型和配置参数，初始化 ROS 2 节点，并设置各种物理参数和 ROS 2 订阅者/发布者。

2. **接收速度命令**

   - 通过订阅 ROS 2 话题(如 `cmd_vel`)，插件能够接收来自外部的速度命令。
   - 收到速度命令后，调用 `OnCmdVel` 回调函数更新目标速度。

3. **仿真更新**

   - 插件注册一个回调函数 `OnUpdate`，每次仿真更新时调用。
   - `OnUpdate` 函数根据目标速度计算每个轮子的速度，并更新机器人的位置和方向。
   - 同时，发布里程计数据和相应的坐标变换，以确保外部 ROS 2 系统能够获取到最新的机器人状态。

4. **里程计数据的发布**

   - 根据编码器数据或仿真世界数据计算里程计信息。
   - 通过 ROS 2 发布里程计消息(如 `odom`)和坐标变换。

### 小结

`GazeboRosDiffDrive` 插件通过继承 `gazebo::ModelPlugin`，并结合 `GazeboRosDiffDrivePrivate` 类，实现了在 Gazebo 仿真环境中对差速驱动机器人的控制。插件通过 ROS 2 进行速度命令的接收和里程计数据的发布，确保机器人能够与 ROS 2 系统进行有效的通信和控制。这种设计模式有效地分离了接口和实现，提高了代码的可维护性和可扩展性。

---

## **You:**

上面的代码介绍了 gazebo 插件，通过 ROS 2 进行速度命令的接收和里程计数据的发布，确保机器人能够与 ROS 2 系统进行有效的通信和控制。
但是我，对于 gazebo 如何通过仿真模型对外发布仿真数据，以及接受外部指令控制模型运动还是不清楚，可以给出详细的说明吗，最好可以给出一些言简意赅的示例代码

---

## **ChatGPT:**

当然，可以详细说明 Gazebo 如何通过仿真模型对外发布仿真数据，并接收外部指令控制模型运动。以下内容将包括一些言简意赅的示例代码，以便更好地理解这个过程。

### Gazebo 通过仿真模型对外发布仿真数据

在 Gazebo 中，可以使用插件(plugin)来获取仿真模型的数据并对外发布。例如，可以获取机器人的位置、速度等数据，并通过 ROS 2 进行发布。

以下是一个简单的示例，展示如何编写一个 Gazebo 插件来发布机器人的位置信息：

#### 示例 1：发布机器人的位置信息

1. **创建一个 Gazebo 插件**

创建一个名为 `robot_pose_plugin.cpp` 的文件，编写以下代码：

```cpp
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace gazebo_plugins {
class RobotPosePlugin : public gazebo::ModelPlugin {
public:
  RobotPosePlugin() : ModelPlugin() {}

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override {
    this->model = _model;
    this->node = rclcpp::Node::make_shared("robot_pose_publisher");

    this->pose_pub = this->node->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose", 10);
    this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&RobotPosePlugin::OnUpdate, this)
    );

    RCLCPP_INFO(this->node->get_logger(), "RobotPosePlugin loaded.");
  }

  void OnUpdate() {
    auto pose = this->model->WorldPose();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->node->now();
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = pose.Pos().X();
    pose_msg.pose.position.y = pose.Pos().Y();
    pose_msg.pose.position.z = pose.Pos().Z();
    pose_msg.pose.orientation.x = pose.Rot().X();
    pose_msg.pose.orientation.y = pose.Rot().Y();
    pose_msg.pose.orientation.z = pose.Rot().Z();
    pose_msg.pose.orientation.w = pose.Rot().W();

    this->pose_pub->publish(pose_msg);
  }

private:
  gazebo::physics::ModelPtr model;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  gazebo::event::ConnectionPtr update_connection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RobotPosePlugin)
}
```

2. **在 SDF 文件中使用插件**

在你的机器人模型的 SDF 文件中，添加如下代码以加载插件：

```xml
<plugin name="robot_pose_plugin" filename="librobot_pose_plugin.so"/>
```

### Gazebo 接受外部指令控制模型运动

Gazebo 可以通过订阅 ROS 2 话题来接收外部指令，从而控制模型的运动。例如，接收 `cmd_vel` 话题的速度命令来控制机器人的移动。

以下是一个简单的示例，展示如何编写一个 Gazebo 插件来接收速度命令并控制机器人的运动：

#### 示例 2：接收速度命令控制机器人运动

1. **创建一个 Gazebo 插件**

创建一个名为 `diff_drive_plugin.cpp` 的文件，编写以下代码：

```cpp
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace gazebo_plugins {
class DiffDrivePlugin : public gazebo::ModelPlugin {
public:
  DiffDrivePlugin() : ModelPlugin() {}

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    this->model = _model;
    this->node = rclcpp::Node::make_shared("diff_drive_controller");

    this->cmd_vel_sub = this->node->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&DiffDrivePlugin::OnCmdVel, this, std::placeholders::_1)
    );

    auto wheel_separation = _sdf->Get<double>("wheel_separation", 1.0).first;
    auto wheel_diameter = _sdf->Get<double>("wheel_diameter", 0.3).first;

    this->left_wheel_joint = _model->GetJoint("left_wheel_joint");
    this->right_wheel_joint = _model->GetJoint("right_wheel_joint");

    this->wheel_radius = wheel_diameter / 2.0;
    this->wheel_separation = wheel_separation;

    RCLCPP_INFO(this->node->get_logger(), "DiffDrivePlugin loaded.");
  }

  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double v = msg->linear.x;
    double w = msg->angular.z;

    double v_left = v - w * this->wheel_separation / 2;
    double v_right = v + w * this->wheel_separation / 2;

    double omega_left = v_left / this->wheel_radius;
    double omega_right = v_right / this->wheel_radius;

    this->left_wheel_joint->SetParam("fmax", 0, 100.0);
    this->left_wheel_joint->SetParam("vel", 0, omega_left);

    this->right_wheel_joint->SetParam("fmax", 0, 100.0);
    this->right_wheel_joint->SetParam("vel", 0, omega_right);
  }

private:
  gazebo::physics::ModelPtr model;
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  gazebo::physics::JointPtr left_wheel_joint;
  gazebo::physics::JointPtr right_wheel_joint;
  double wheel_radius;
  double wheel_separation;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DiffDrivePlugin)
}
```

2. **在 SDF 文件中使用插件**

在你的机器人模型的 SDF 文件中，添加如下代码以加载插件，并设置参数：

```xml
<plugin name="diff_drive_plugin" filename="libdiff_drive_plugin.so">
  <wheel_separation>1.25</wheel_separation>
  <wheel_diameter>0.6</wheel_diameter>
</plugin>
```

### 总结

通过上述示例，可以看到 Gazebo 插件如何与 ROS 2 集成来发布仿真数据和接收控制指令。这两个示例分别展示了如何发布机器人的位置信息以及如何接收速度命令来控制机器人的运动。这样的插件设计可以帮助你在仿真环境中测试和验证机器人的控制算法。

---

## **You:**

如果在 gazebo 仿真中的对象是一个复杂的机器人对象，是一个四足机器人具有 12 个关节，应该如何获取这个机器人中的每个关节的状态，以及向每个关节发送运动指令？

---

## **ChatGPT:**

如果你在 Gazebo 中仿真的是一个复杂的四足机器人(具有 12 个关节)，你可以编写一个 Gazebo 插件来获取每个关节的状态，并向每个关节发送运动指令。下面的示例代码展示了如何实现这一点。

### 获取关节状态和发送运动指令

1. **创建一个 Gazebo 插件**

首先，创建一个 Gazebo 插件来处理你的四足机器人。假设每条腿有 3 个关节，总共有 12 个关节。我们需要获取这些关节的状态，并能向这些关节发送控制命令。

```cpp
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace gazebo_plugins {

class QuadrupedController : public gazebo::ModelPlugin {
public:
  QuadrupedController() : ModelPlugin() {}

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    this->model = _model;
    this->node = rclcpp::Node::make_shared("quadruped_controller");

    // Initialize joint pointers
    for (const auto& joint_name : {"joint1", "joint2", "joint3", "joint4",
                                   "joint5", "joint6", "joint7", "joint8",
                                   "joint9", "joint10", "joint11", "joint12"}) {
      auto joint = _model->GetJoint(joint_name);
      if (joint) {
        joints.push_back(joint);
      } else {
        RCLCPP_ERROR(this->node->get_logger(), "Joint %s not found!", joint_name);
      }
    }

    // Initialize ROS publisher for joint states
    this->joint_state_pub = this->node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Initialize ROS subscriber for joint commands
    this->joint_cmd_sub = this->node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "joint_commands", 10, std::bind(&QuadrupedController::OnJointCommand, this, std::placeholders::_1)
    );

    // Connect to the Gazebo update event
    this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&QuadrupedController::OnUpdate, this)
    );

    RCLCPP_INFO(this->node->get_logger(), "QuadrupedController loaded.");
  }

  void OnUpdate() {
    // Publish joint states
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->node->now();
    for (const auto& joint : joints) {
      joint_state_msg.name.push_back(joint->GetName());
      joint_state_msg.position.push_back(joint->Position(0));
      joint_state_msg.velocity.push_back(joint->GetVelocity(0));
      joint_state_msg.effort.push_back(joint->GetForce(0));
    }
    this->joint_state_pub->publish(joint_state_msg);
  }

  void OnJointCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != joints.size()) {
      RCLCPP_ERROR(this->node->get_logger(), "Joint command size mismatch!");
      return;
    }

    // Apply joint commands
    for (size_t i = 0; i < joints.size(); ++i) {
      joints[i]->SetParam("fmax", 0, 100.0); // Set maximum force
      joints[i]->SetParam("vel", 0, msg->data[i]); // Set velocity
    }
  }

private:
  gazebo::physics::ModelPtr model;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub;
  std::vector<gazebo::physics::JointPtr> joints;
  gazebo::event::ConnectionPtr update_connection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(QuadrupedController)
}
```

2. **在 SDF 文件中使用插件**

在你的四足机器人模型的 SDF 文件中，添加如下代码以加载插件，并确保正确命名关节：

```xml
<plugin name="quadruped_controller" filename="libquadruped_controller.so">
</plugin>
```

### 总结

通过上述代码，你可以实现一个 Gazebo 插件来获取四足机器人每个关节的状态，并通过 ROS 2 接收控制指令来控制这些关节的运动。这个插件实现了一个基本的关节状态发布机制和命令接收机制，可以根据你的具体需求进行扩展和优化。

- **获取关节状态**：通过 `OnUpdate` 方法定期获取每个关节的位置、速度和力矩，并通过 ROS 2 话题发布这些状态。
- **发送运动指令**：通过 `OnJointCommand` 方法接收 ROS 2 话题发布的关节速度命令，并应用到相应的关节上。

这种插件设计能够帮助你在 Gazebo 仿真环境中实现复杂机器人的控制与状态监控。

---

## **You:**

你在上述示例中通过：

```cpp
      joints[i]->SetParam("fmax", 0, 100.0); // Set maximum force
      joints[i]->SetParam("vel", 0, msg->data[i]); // Set velocity
```

将关节的速度值设置到 gazebo 仿真机器人中，如果希望控制关节的力矩或者控制关节的位置，应该如何修改？这些设置接口的 API 应该如何获取，有对应的文档手册吗？

---

## **ChatGPT:**

要控制 Gazebo 仿真机器人关节的力矩或位置，需要使用 Gazebo 提供的不同接口和方法。以下是如何修改代码以控制关节的力矩或位置，并介绍如何查找相关 API 文档。

### 控制关节的力矩

要设置关节的力矩，可以使用 `SetForce` 方法。这直接设置关节的力矩。

```cpp
void OnJointCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() != joints.size()) {
    RCLCPP_ERROR(this->node->get_logger(), "Joint command size mismatch!");
    return;
  }

  // Apply joint commands
  for (size_t i = 0; i < joints.size(); ++i) {
    joints[i]->SetForce(0, msg->data[i]); // Set force/torque
  }
}
```

### 控制关节的位置

要控制关节的位置，通常使用 Gazebo 的 PID 控制器。你需要设置目标位置并使用 PID 控制器计算所需的力矩或速度。

首先，初始化 PID 控制器：

```cpp
#include <gazebo/common/PID.hh>

std::vector<gazebo::common::PID> pids;

void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
  this->model = _model;
  this->node = rclcpp::Node::make_shared("quadruped_controller");

  // Initialize joint pointers and PID controllers
  for (const auto& joint_name : {"joint1", "joint2", "joint3", "joint4",
                                 "joint5", "joint6", "joint7", "joint8",
                                 "joint9", "joint10", "joint11", "joint12"}) {
    auto joint = _model->GetJoint(joint_name);
    if (joint) {
      joints.push_back(joint);
      pids.emplace_back(0.1, 0, 0); // PID parameters: P, I, D
      this->model->GetJointController()->SetPositionPID(joint->GetScopedName(), pids.back());
    } else {
      RCLCPP_ERROR(this->node->get_logger(), "Joint %s not found!", joint_name);
    }
  }

  // Other initializations...
}
```

然后，在接收命令时设置目标位置：

```cpp
void OnJointCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() != joints.size()) {
    RCLCPP_ERROR(this->node->get_logger(), "Joint command size mismatch!");
    return;
  }

  // Apply joint position commands
  for (size_t i = 0; i < joints.size(); ++i) {
    this->model->GetJointController()->SetPositionTarget(joints[i]->GetScopedName(), msg->data[i]);
  }
}
```

### 查找 API 文档

要获取 Gazebo 的 API 文档，可以参考以下资源：

1. **Gazebo 官方文档**：

   - Gazebo 的官方文档提供了全面的 API 参考，可以在 Gazebo API Documentation 中找到。
   - 具体到关节控制，可以查阅 Joint API。

2. **Gazebo 论坛和社区**：

   - Gazebo 论坛和社区是获取帮助和了解最佳实践的好地方。访问 Gazebo Answers 提问或搜索相关问题。

3. **ROS 2 和 Gazebo 集成**：

   - 了解 ROS 2 和 Gazebo 的集成，可以参考 [ros-simulation/gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs) 仓库中的示例和文档。

### 示例总结

完整的示例代码如下：

```cpp
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <gazebo/common/PID.hh>

namespace gazebo_plugins {

class QuadrupedController : public gazebo::ModelPlugin {
public:
  QuadrupedController() : ModelPlugin() {}

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    this->model = _model;
    this->node = rclcpp::Node::make_shared("quadruped_controller");

    // Initialize joint pointers and PID controllers
    for (const auto& joint_name : {"joint1", "joint2", "joint3", "joint4",
                                   "joint5", "joint6", "joint7", "joint8",
                                   "joint9", "joint10", "joint11", "joint12"}) {
      auto joint = _model->GetJoint(joint_name);
      if (joint) {
        joints.push_back(joint);
        pids.emplace_back(0.1, 0, 0); // PID parameters: P, I, D
        this->model->GetJointController()->SetPositionPID(joint->GetScopedName(), pids.back());
      } else {
        RCLCPP_ERROR(this->node->get_logger(), "Joint %s not found!", joint_name);
      }
    }

    // Initialize ROS publisher for joint states
    this->joint_state_pub = this->node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Initialize ROS subscriber for joint commands
    this->joint_cmd_sub = this->node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "joint_commands", 10, std::bind(&QuadrupedController::OnJointCommand, this, std::placeholders::_1)
    );

    // Connect to the Gazebo update event
    this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&QuadrupedController::OnUpdate, this)
    );

    RCLCPP_INFO(this->node->get_logger(), "QuadrupedController loaded.");
  }

  void OnUpdate() {
    // Publish joint states
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->node->now();
    for (const auto& joint : joints) {
      joint_state_msg.name.push_back(joint->GetName());
      joint_state_msg.position.push_back(joint->Position(0));
      joint_state_msg.velocity.push_back(joint->GetVelocity(0));
      joint_state_msg.effort.push_back(joint->GetForce(0));
    }
    this->joint_state_pub->publish(joint_state_msg);
  }

  void OnJointCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != joints.size()) {
      RCLCPP_ERROR(this->node->get_logger(), "Joint command size mismatch!");
      return;
    }

    // Apply joint position commands
    for (size_t i = 0; i < joints.size(); ++i) {
      this->model->GetJointController()->SetPositionTarget(joints[i]->GetScopedName(), msg->data[i]);
    }
  }

private:
  gazebo::physics::ModelPtr model;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub;
  std::vector<gazebo::physics::JointPtr> joints;
  std::vector<gazebo::common::PID> pids;
  gazebo::event::ConnectionPtr update_connection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(QuadrupedController)
}
```

通过这个示例，你可以根据需要设置关节的力矩或位置，并利用 PID 控制器实现精确控制。这些方法可以帮助你更好地与 Gazebo 仿真进行交互和控制。
