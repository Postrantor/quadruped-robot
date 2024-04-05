/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Gazebo plugin for ros_control that allows 'hardware_interfaces' to be plugged in
           using pluginlib
*/

#ifndef GAZEBO_ROS2_CONTROL__GAZEBO_ROS2_CONTROL_PLUGIN_HPP_
#define GAZEBO_ROS2_CONTROL__GAZEBO_ROS2_CONTROL_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_manager/controller_manager.hpp"

#include "gazebo/common/common.hh"
#include "gazebo/physics/Model.hh"

namespace gazebo_ros2_control {
class GazeboRosControlPrivate;

class GazeboRosControlPlugin : public gazebo::ModelPlugin {
public:
  GazeboRosControlPlugin();

  ~GazeboRosControlPlugin();

  // Overloaded Gazebo entry point
  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosControlPrivate> impl_;
};
}  // namespace gazebo_ros2_control

#endif  // GAZEBO_ROS2_CONTROL__GAZEBO_ROS2_CONTROL_PLUGIN_HPP_
