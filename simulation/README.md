---
date: 2024-07-21 22:10:40
---

## start

```bash
ros2 launch robot_description gazebo.launch.py --debug
ros2 control list_controllers -v
```

###

尝试通过命令行方式发送控制指令可能会有问题？
可能是以为数据类型不完全匹配的问题，需要看一下 controller 的细节。

```bash
ros2 topic pub --rate 1 /FL_calf_controller/desired_state geometry_msgs/msg/TwistStamped "{
  twist: {
    linear: {x: 0.7, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 1.0},
  }
}"

ros2 topic pub --rate 1 /FL_calf_controller/desired_cmd unitree_msgs/msg/MotorCmd "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'frame_id_string'
  },
  hex_len: 17,
  id: 0,
  mode: 10,
  tau: 2.0,
  dq: 2.0,
  q: 15.0,
  k_q: 5.0,
  k_dq: 5.0
}"
```

### gazebo 调试

默认起一个 gazebo server 等价于如下指令，会加载一些 so：

```bash
> gzserver /home/trantor/project/model_ws/install/robot_description/share/robot_description/worlds/earth.world -slibgazebo_ros_init.so -slibgazebo_ros_factory.so -slibgazebo_ros_force_system.so

> ros2 service list
/spawn_entity
/get_model_list
/apply_joint_effort
/apply_link_wrench
/clear_joint_efforts
/clear_link_wrenches
/delete_entity
/reset_simulation
/reset_world
/pause_physics
/unpause_physics

/gazebo/describe_parameters
/gazebo/get_parameter_types
/gazebo/get_parameters
/gazebo/list_parameters
/gazebo/set_parameters
/gazebo/set_parameters_atomically
```

## simuliation 文件结构

- "gazebo_plugins": 用于模拟传感器等逐渐的第三方插件，由 gazebo 官方提供；
- "gazebo_ros":
- "qr_gazebo": 支持 ROS1
  - launcher: 提供 launcher 文件，加载相应组件；
  - controller_manager: 提供控制不同 controller 的组件；
  - plugin: 提供辅助插件，用于在 xpp 中绘制受力关系；
- "robot_description": 基于 ROS2 的功能包，用于将模型加载到 gazebo 中；
- 'xpp': 支持 ROS1，用于在 rviz 界面中绘制足端受力情况，需要 gazebo plugin 相应插件支持，由 "qr_gazebo" 插件提供；

### gazebo_ros_pkgs

用于与 Gazebo 接口的 ROS 2 包集包含在名为 gazebo_ros_pkgs 的元包中。在继续此处之前，请参阅 ROS 2 概述以获取背景信息。

[gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2)
[demo](https://github.com/ros-simulation/gazebo_ros_demos/tree/foxy)
[tutorials](https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)

支持 ROS2 开发，包含的功能包：

- "gazebo_ros_pkgs": 元功能包；
- "gazebo_dev": gazebo 依赖功能包，用于编译的功能包；

- "gazebo_msgs" : interfaces 定义；

- "gazebo_plugins" 用于控制 gazebo 中对象的第三方插件，如 IMU 传感器等，包示例的括机器人主体，有 diff_drive_robot
  其中一些 IMU/Position 等组件还是可以用的；
- "gazebo_ros": 用于 ROS2 <-> Gazebo 交互层，编译的产物为 so，通过插件的方式集成；
- "gazebo_ros_control": 依赖功能包，这是一个 ROS 2 软件包，用于将 ros2_control 控制器架构与 Gazebo Classic 模拟器集成；
  该功能包使用的 API 与 gazebo_ros 相同，应该是等价关系；

关于该功能包的迁移，可以参考：

![](simulation/gazebo_demos/README.md)
