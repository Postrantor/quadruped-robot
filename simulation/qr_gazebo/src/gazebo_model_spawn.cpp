/**
 * @author GPT4-o
 * @brief
 * @date 2024-06-23 02:58:50
 * @copyright Copyright (c) 2024
 */

#include "qr_gazebo/gazebo_model_spawn.h"

#include <vector>
#include <string>

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

bool GazeboSpawner::spawn_model(std::string urdf_param) {
  int state = system(std::string(
                         "ros2 run gazebo_ros spawn_entity.py -entity " + d_robot_type + "_gazebo -topic " +
                         urdf_param + " -x 0 -y 0 -z 0.4")
                         .c_str());
  RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "spawn model state: %d", state);

  sleep(2);
  RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "Gazebo model spawn correctly");

  return true;
}

bool GazeboSpawner::delete_model() {
  int state = system(std::string(
                         "ros2 service call /gazebo/delete_entity gazebo_msgs/srv/DeleteEntity \"{name: '" +
                         d_robot_type + "_gazebo'}\"")
                         .c_str());
  RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "delete model state: %d", state);

  RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "Gazebo model delete correctly");

  return true;
}

void GazeboSpawner::start_controllers() {
  std::string service_name = "/" + d_robot_type + "_gazebo" + "/controller_manager/switch_controller";
  auto client = d_node->create_client<controller_manager_msgs::srv::SwitchController>(service_name);

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->start_controllers = controller_list;
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

  auto result = client->async_send_request(request);
  if (client->wait_for_service(std::chrono::seconds(1))) {
    if (rclcpp::spin_until_future_complete(d_node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "success to call start controllers service");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("gazebo_model_spawn"), "fail to call start controllers service");
    }
  }

  while (!result.get()->ok) {
    RCLCPP_ERROR(rclcpp::get_logger("gazebo_model_spawn"), "Starting controllers...");
  }

  RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "Controllers start correctly");
}

bool GazeboSpawner::stop_controllers() {
  std::string service_name = "/" + d_robot_type + "_gazebo" + "/controller_manager/switch_controller";
  auto client = d_node->create_client<controller_manager_msgs::srv::SwitchController>(service_name);

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->stop_controllers = controller_list;
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

  auto result = client->async_send_request(request);
  if (client->wait_for_service(std::chrono::seconds(1))) {
    if (rclcpp::spin_until_future_complete(d_node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "success to call stop controllers service");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("gazebo_model_spawn"), "fail to call stop controllers service");
    }
  }

  while (!result.get()->ok) {
    RCLCPP_WARN(rclcpp::get_logger("gazebo_model_spawn"), "Stopping controllers...");
  }
  RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "Controllers stopped correctly!");
  return true;
}

void GazeboSpawner::load_controllers() {
  for (const auto& element : controller_list) {
    load_controller_once(element);
  }
}

void GazeboSpawner::unload_controllers() {
  for (const auto& element : controller_list) {
    unload_controller_once(element);
  }
}

void GazeboSpawner::load_controller_once(const std::string& controller_name) {
  std::string service_name = "/" + d_robot_type + "_gazebo" + "/controller_manager/load_controller";
  auto client = d_node->create_client<controller_manager_msgs::srv::LoadController>(service_name);

  auto request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
  request->name = controller_name;

  auto result = client->async_send_request(request);
  if (client->wait_for_service(std::chrono::seconds(1))) {
    if (rclcpp::spin_until_future_complete(d_node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "success to call load_controller service");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("gazebo_model_spawn"), "fail to call load_controller service");
    }
  }

  while (!result.get()->ok) {
    RCLCPP_WARN(rclcpp::get_logger("gazebo_model_spawn"), "Loading controllers...");
  }
  RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "%s load correctly", controller_name.c_str());
}

void GazeboSpawner::unload_controller_once(const std::string& controller_name) {
  std::string service_name = "/" + d_robot_type + "_gazebo" + "/controller_manager/unload_controller";
  auto client = d_node->create_client<controller_manager_msgs::srv::UnloadController>(service_name);

  auto request = std::make_shared<controller_manager_msgs::srv::UnloadController::Request>();
  request->name = controller_name;

  auto result = client->async_send_request(request);
  if (client->wait_for_service(std::chrono::seconds(1))) {
    if (rclcpp::spin_until_future_complete(d_node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "success to call unload_controller service");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("gazebo_model_spawn"), "fail to call unload_controller service");
    }
  }

  while (!result.get()->ok) {
    RCLCPP_WARN(rclcpp::get_logger("gazebo_model_spawn"), "Unloading controllers...");
  }
  RCLCPP_INFO(rclcpp::get_logger("gazebo_model_spawn"), "%s unload correctly", controller_name.c_str());
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
