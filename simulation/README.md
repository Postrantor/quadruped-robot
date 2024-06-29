---
date: 2024-06-23 01:12:04
---

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
- "gazebo_ros": 用于 ROS2 <-> Gazebo 交互层，编译的产物为so，通过插件的方式集成；
- "gazebo_ros_control": 依赖功能包，这是一个 ROS 2 软件包，用于将 ros2_control 控制器架构与 Gazebo Classic 模拟器集成；
  该功能包使用的 API 与 gazebo_ros 相同，应该是等价关系；

关于该功能包的迁移，可以参考：

![](simulation/gazebo_demos/README.md)
