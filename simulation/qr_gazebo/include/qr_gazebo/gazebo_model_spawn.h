/**
 * @author GPT4-o
 * @brief
 * @date 2024-06-23 02:59:08
 * @copyright Copyright (c) 2024
 */

#ifndef GAZEBO_MODEL_SPAWN_H
#define GAZEBO_MODEL_SPAWN_H

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/unload_controller.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"

class GazeboSpawner {
public:
  GazeboSpawner(const std::string robot_type, std::shared_ptr<rclcpp::Node> node)
      : d_robot_type(robot_type), d_node(node) {
    ;
  }

  bool spawn_model(std::string urdf_param);
  bool delete_model();
  void start_controllers();
  bool stop_controllers();
  void load_controllers();
  void unload_controllers();
  void set_model_position(double x, double y, double z);
  void set_model_orientation(double w, double x, double y, double z);

protected:
  static const std::vector<std::string> controller_list;
  static geometry_msgs::msg::Pose model_pose;

private:
  std::string d_robot_type;
  std::shared_ptr<rclcpp::Node> d_node;

  void load_controller_once(const std::string& controller_name);
  void unload_controller_once(const std::string& controller_name);
};

#endif  // GAZEBO_MODEL_SPAWN_H
