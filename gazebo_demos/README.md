## how to run

```bash
ros2 launch gazebo_ros2_control_demos robot.launch.py
ros2 run gazebo_ros2_control_demos example_robot
```

## topic/service

在通过 launch 启动 gazebo 之后，可以在终端中查看到如下的 Topic/Service

### topic

```bash
trantor ➜ ~/project/demo_ws

> ros2 topic list
# 用于和controller通信的topic
/diff_drive_base_controller/cmd_vel_unstamped
/diff_drive_base_controller/odom
/diff_drive_base_controller/transition_event

/dynamic_joint_states
/joint_state_broadcaster/transition_event
/joint_states

/parameter_events
/performance_metrics

/robot_description

/rosout
/tf
/tf_static
```

### service

通过 `ros2 service type <service_name>` 查看对应服务类型，按照相同的类型对比 example 实例程序中的 srv

#### ROS2 --> ROS1 迁移相关文档

在 ROS1 -> ROS2 的过程中有一些服务被拆分了，所以这部分代码在移植过程中也要注意，以下是 gazebo_ros 包中的一些相关讨论和发布内容：

[ros2 gazebo server list](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/779)
[gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_ros/CMakeLists.txt)

[gazebo_ros_api_plugin](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-gazebo_ros_api_plugin)
[Spawn and delete](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Spawn-and-delete)
[Set_model_configuration service ported to ROS 2](https://discourse.ros.org/t/set-model-configuration-service-ported-to-ros-2-gazebo-ros/32995)

以下链接是 ROS1 中 gazebo_ros 提供的服务列表示例：

![](ascend/model-control/README.md)

#### 目前使用的仿真中默认包含的 service

```bash
trantor ➜ ~/project/demo_ws

> ros2 service list
## support by `gazebo_ros_force_system`
/apply_joint_effort
  # > gazebo_msgs/srv/ApplyJointEffort
/apply_link_wrench
  # > gazebo_msgs/srv/ApplyLinkWrench
/clear_joint_efforts
  # > gazebo_msgs/srv/JointRequest
/clear_link_wrenches
  # > gazebo_msgs/srv/LinkRequest

## support by `gazebo_ros_factory`
# [ROS 2 Migration: Spawn and delete](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Spawn-and-delete)
/delete_entity
  # > gazebo_msgs/srv/DeleteEntity
/spawn_entity
  # > gazebo_msgs/srv/SpawnEntity

/get_model_list
  # > gazebo_msgs/srv/GetModelList

## support by `gazebo_ros_init`
/reset_simulation
  # > std_srvs/srv/Empty
/reset_world
  # > std_srvs/srv/Empty
/pause_physics
  # > std_srvs/srv/Empty
/unpause_physics
  # > std_srvs/srv/Empty

/controller_manager/describe_parameters
  # > rcl_interfaces/srv/DescribeParameters
/controller_manager/get_parameter_types
  # > rcl_interfaces/srv/GetParameterTypes
/controller_manager/get_parameters
  # > rcl_interfaces/srv/GetParameters
/controller_manager/list_parameters
  # > rcl_interfaces/srv/ListParameters
/controller_manager/set_parameters
  # > rcl_interfaces/srv/SetParameters
/controller_manager/set_parameters_atomically
  # > rcl_interfaces/srv/SetParametersAtomically

/controller_manager/configure_controller
  # > controller_manager_msgs/srv/ConfigureController
/controller_manager/list_controller_types
  # > controller_manager_msgs/srv/ListControllerTypes
/controller_manager/list_controllers
  # > controller_manager_msgs/srv/ListControllers
/controller_manager/list_hardware_components
  # > controller_manager_msgs/srv/ListHardwareComponents
/controller_manager/list_hardware_interfaces
  # > controller_manager_msgs/srv/ListHardwareInterfaces
/controller_manager/load_controller
  # > controller_manager_msgs/srv/LoadController
/controller_manager/reload_controller_libraries
  # > controller_manager_msgs/srv/ReloadControllerLibraries
/controller_manager/set_hardware_component_state
  # > controller_manager_msgs/srv/SetHardwareComponentState
/controller_manager/switch_controller
  # > controller_manager_msgs/srv/SwitchController
/controller_manager/unload_controller
  # > controller_manager_msgs/srv/UnloadController

/diff_drive_base_controller/describe_parameters
  # > rcl_interfaces/srv/DescribeParameters
/diff_drive_base_controller/get_parameter_types
  # > rcl_interfaces/srv/GetParameterTypes
/diff_drive_base_controller/get_parameters
  # > rcl_interfaces/srv/GetParameters
/diff_drive_base_controller/list_parameters
  # > rcl_interfaces/srv/ListParameters
/diff_drive_base_controller/set_parameters
  # > rcl_interfaces/srv/SetParameters
/diff_drive_base_controller/set_parameters_atomically
  # > rcl_interfaces/srv/SetParametersAtomically

/gazebo/describe_parameters
/gazebo/get_parameter_types
/gazebo/get_parameters
/gazebo/list_parameters
/gazebo/set_parameters
/gazebo/set_parameters_atomically

/gazebo_ros2_control/describe_parameters
/gazebo_ros2_control/get_parameter_types
/gazebo_ros2_control/get_parameters
/gazebo_ros2_control/list_parameters
/gazebo_ros2_control/set_parameters
/gazebo_ros2_control/set_parameters_atomically

/joint_state_broadcaster/describe_parameters
/joint_state_broadcaster/get_parameter_types
/joint_state_broadcaster/get_parameters
/joint_state_broadcaster/list_parameters
/joint_state_broadcaster/set_parameters
/joint_state_broadcaster/set_parameters_atomically

/robot_state_publisher/describe_parameters
/robot_state_publisher/get_parameter_types
/robot_state_publisher/get_parameters
/robot_state_publisher/list_parameters
/robot_state_publisher/set_parameters
/robot_state_publisher/set_parameters_atomically
```

#### 对比 ROS1 中仿真缺少的 servers

##### 插件 `gazebo_ros_state`

- service: get_entity_state
- service: set_entity_state
- topic: model_states

这是一个世界插件，其中：

- Provides the get_entity_state ROS service for querying the pose and twist of models, links and lights in simulation.
- Provides the set_entity_state ROS service for setting the pose and twist of models, links and lights in simulation.
- Publishes the model_states ROS topic periodically with the pose and twist of all models in simulation.

- 提供 ROS 服务，get_entity_state 用于查询仿真中模型、链接和灯光的姿态和扭转情况。
- 提供 ROS 服务，set_entity_state 用于在仿真中设置模型、链接和灯光的姿势和扭曲。
- 定期发布 model_states ROS 主题，其中包含仿真中所有模型的姿态和扭转。

See more details on ROS 2 Migration: [Entity states](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Entity-states)
