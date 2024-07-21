/**
 * @brief Gazebo模型生成器类的声明
 * @details 该类提供了在Gazebo仿真环境中生成、删除和管理机器人模型的方法。
 * @date 2024-06-23 10:47:49
 */

#ifndef QR_GAZEBO_GAZEBO_MODEL_SPAWN_H
#define QR_GAZEBO_GAZEBO_MODEL_SPAWN_H

#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/unload_controller.hpp"
#include "geometry_msgs/msg/pose.hpp"

class GazeboSpawner {
public:
  GazeboSpawner(
      const std::string& robot_type,  //
      const rclcpp::Node::SharedPtr& node)
      : d_robot_type(robot_type),  //
        d_node(node) {}

  std::string get_robot_type() const { return d_robot_type; }

  rclcpp::Node::SharedPtr get_node() const { return d_node; }

  /**
   * @brief 生成模型
   * @details 调用外部命令行工具将URDF文件中的机器人模型生成到Gazebo仿真环境中。
   * @param urdf_param URDF文件参数
   * @return 成功生成模型返回true，否则返回false
   */
  bool spawn_model(const std::string& urdf_param);

  /**
   * @brief 删除模型
   * @details 调用服务接口删除Gazebo仿真环境中的机器人模型。
   * @return 成功删除模型返回true，否则返回false
   */
  bool delete_model();

  /**
   * @brief 启动控制器
   * @details 调用服务接口启动机器人模型的控制器。
   */
  void start_controllers();

  /**
   * @brief 停止控制器
   * @details 调用服务接口停止机器人模型的控制器。
   * @return 成功停止控制器返回true，否则返回false
   */
  bool stop_controllers();

  /**
   * @brief 加载所有控制器
   * @details 调用服务接口加载控制器列表中的所有控制器。
   */
  void load_controllers();

  /**
   * @brief 卸载所有控制器
   * @details 调用服务接口卸载控制器列表中的所有控制器。
   */
  void unload_controllers();

  /**
   * @brief 设置模型位置
   * @details 设置模型在Gazebo仿真环境中的位置。
   * @param x X轴坐标
   * @param y Y轴坐标
   * @param z Z轴坐标
   */
  void set_model_position(double x, double y, double z);

  /**
   * @brief 设置模型朝向
   * @details 设置模型在Gazebo仿真环境中的朝向（四元数表示）。
   * @param w 四元数w分量
   * @param x 四元数x分量
   * @param y 四元数y分量
   * @param z 四元数z分量
   */
  void set_model_orientation(double w, double x, double y, double z);

  static const std::vector<std::string> controller_list;
  static geometry_msgs::msg::Pose model_pose;

private:
  std::string d_robot_type;
  rclcpp::Node::SharedPtr d_node;

  /**
   * @brief 加载单个控制器
   * @details 调用服务接口加载指定名称的控制器。
   * @param controller_name 控制器名称
   */
  void load_controller_once(const std::string& controller_name);

  /**
   * @brief 卸载单个控制器
   * @details 调用服务接口卸载指定名称的控制器。
   * @param controller_name 控制器名称
   */
  void unload_controller_once(const std::string& controller_name);

  /**
   * @brief 通用服务调用函数的实现
   * @details 调用指定服务接口并处理结果。
   * @tparam ServiceT 服务类型
   * @tparam RequestT 请求类型
   * @param service_name 服务名称
   * @param request 服务请求
   */
  template <typename ServiceT, typename RequestT>
  void call_service(const std::string& service_name, std::shared_ptr<RequestT> request) {
    auto client = d_node->create_client<ServiceT>(service_name);

    if (client->wait_for_service(std::chrono::seconds(1))) {
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(d_node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        if (result.get()->ok) {
          RCLCPP_INFO_STREAM(d_node->get_logger(), "Successfully called service: " << service_name.c_str());
        } else {
          RCLCPP_WARN_STREAM(d_node->get_logger(), "Service call failed: " << service_name.c_str());
        }
      } else {
        RCLCPP_ERROR_STREAM(d_node->get_logger(), "Failed to call service: " << service_name.c_str());
      }
    } else {
      RCLCPP_ERROR_STREAM(d_node->get_logger(), "Service not available: " << service_name.c_str());
    }
  }
};

#endif  // QR_GAZEBO_GAZEBO_MODEL_SPAWN_H
