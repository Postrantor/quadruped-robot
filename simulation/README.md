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
