/**
 * @brief Gazebo模型生成器类的实现
 * @detail 该类提供了在Gazebo仿真环境中生成、删除和管理机器人模型的方法。
 * @date 2024-06-23 02:58:50
 */

#include "qr_gazebo/gazebo_model_spawn.hpp"

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"

const std::vector<std::string> GazeboSpawner::controller_list = {
    "joint_state_controller",  //
    "FR_hip_controller",       //
    "FR_thigh_controller",     //
    "FR_calf_controller",      //
    "FL_hip_controller",       //
    "FL_thigh_controller",     //
    "FL_calf_controller",      //
    "RR_hip_controller",       //
    "RR_thigh_controller",     //
    "RR_calf_controller",      //
    "RL_hip_controller",       //
    "RL_thigh_controller",     //
    "RL_calf_controller"};

geometry_msgs::msg::Pose GazeboSpawner::model_pose = geometry_msgs::msg::Pose();

bool GazeboSpawner::spawn_model(const std::string& urdf_param) {
  auto command = "ros2 run gazebo_ros spawn_entity.py -entity " + d_robot_type + "_gazebo -topic " + urdf_param +
                 " -x 0 -y 0 -z 0.4";
  int state = system(command.c_str());
  RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "spawn model state: %d", state);

  std::this_thread::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "gazebo model spawn correctly");

  return state == 0;
}

bool GazeboSpawner::delete_model() {
  auto command =
      "ros2 service call /gazebo/delete_entity gazebo_msgs/srv/DeleteEntity \"{name: '" + d_robot_type + "_gazebo'}\"";
  int state = system(command.c_str());
  RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "delete model state: %d", state);
  RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "gazebo model delete correctly");

  return state == 0;
}

void GazeboSpawner::start_controllers() {
  auto service_name = "/" + d_robot_type + "_gazebo/controller_manager/switch_controller";
  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->start_controllers = controller_list;
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

  call_service<controller_manager_msgs::srv::SwitchController>(service_name, request);
}

bool GazeboSpawner::stop_controllers() {
  auto service_name = "/" + d_robot_type + "_gazebo/controller_manager/switch_controller";
  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->stop_controllers = controller_list;
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

  call_service<controller_manager_msgs::srv::SwitchController>(service_name, request);
  return true;
}

void GazeboSpawner::load_controllers() {
  for (const auto& controller_name : controller_list) {
    load_controller_once(controller_name);
  }
}

void GazeboSpawner::unload_controllers() {
  for (const auto& controller_name : controller_list) {
    unload_controller_once(controller_name);
  }
}

void GazeboSpawner::load_controller_once(const std::string& controller_name) {
  auto service_name = "/" + d_robot_type + "_gazebo/controller_manager/load_controller";
  auto request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
  request->name = controller_name;

  call_service<controller_manager_msgs::srv::LoadController>(service_name, request);
}

void GazeboSpawner::unload_controller_once(const std::string& controller_name) {
  auto service_name = "/" + d_robot_type + "_gazebo/controller_manager/unload_controller";
  auto request = std::make_shared<controller_manager_msgs::srv::UnloadController::Request>();
  request->name = controller_name;

  call_service<controller_manager_msgs::srv::UnloadController>(service_name, request);
}

void GazeboSpawner::set_model_position(double x, double y, double z) {
  model_pose.position.x = x;
  model_pose.position.y = y;
  model_pose.position.z = z;
}

void GazeboSpawner::set_model_orientation(double w, double x, double y, double z) {
  model_pose.orientation.w = w;
  model_pose.orientation.x = x;
  model_pose.orientation.y = y;
  model_pose.orientation.z = z;
}
