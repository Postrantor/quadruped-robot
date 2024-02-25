---
tip: translate by baidu@2024-02-12 19:30:11
github_url: |
  <https://github.com/ros-controls/ros2_controllers/blob/%7BREPOS_FILE_BRANCH%7D/doc/controllers_index.rst>
---

# ros2_controllers {#controllers}

Commonly used and generalized controllers for ros2_control framework that are ready to use with many robots, [MoveIt2](https://moveit.picknik.ai/main/index.html) and [Nav2](https://navigation.ros.org/).

> ros2_control 框架的常用和通用控制器，可用于许多机器人[MoveIt2](https://moveit.picknik.ai/main/index.html)和[Nav2](https://navigation.ros.org/).

[Link to GitHub Repository](https://github.com/ros-controls/ros2_controllers)

> [链接到 GitHub 存储库](https://github.com/ros-controls/ros2_controllers)

## Guidelines and Best Practices

::: {.toctree titlesonly="" glob=""}

- :::

## Controllers for Mobile Robots

::: {.toctree titlesonly=""}

Ackermann Steering Controller \<../ackermann_steering_controller/doc/userdoc.rst\> Bicycle Steering Controller \<../bicycle_steering_controller/doc/userdoc.rst\> Differential Drive Controller \<../diff_drive_controller/doc/userdoc.rst\> Steering Controllers Library \<../steering_controllers_library/doc/userdoc.rst\> Tricycle Controller \<../tricycle_controller/doc/userdoc.rst\> Tricycle Steering Controller \<../tricycle_steering_controller/doc/userdoc.rst\>

> 阿克曼转向控制器/ackermann_steering_controller/doc/userdoc.rst\>自行车转向控制器\</bicycle_steering_controller/doc/userdoc.rst\>差速驱动控制器\</diff_drive_controller/doc/userdoc.rst\>转向控制器库\</steering_controllers_brary/doc/userdoc.rst\>三轮车控制器\</三轮车转向控制器/trigger_steering_controller/doc/userdoc.rst\>
> :::

## Controllers for Manipulators and Other Robots

The controllers are using [common hardware interface definitions](https://github.com/ros-controls/ros2_control/blob/%7BREPOS_FILE_BRANCH%7D/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp), and may use namespaces depending on the following command interface types:

> 控制器使用[通用硬件接口定义](https://github.com/ros-controls/ros2_control/blob/%7BREPOS_FILE_BRANCH%7D/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp)，并且可以使用命名空间，具体取决于以下命令接口类型：

> - `position_controllers`: `hardware_interface::HW_IF_POSITION`
> - `velocity_controller`: `hardware_interface::HW_IF_VELOCITY`
> - `effort_controllers`: `hardware_interface::HW_IF_ACCELERATION`
> - `effort_controllers`: `hardware_interface::HW_IF_EFFORT`

::: {.toctree titlesonly=""}

Admittance Controller \<../admittance_controller/doc/userdoc.rst\> Effort Controllers \<../effort_controllers/doc/userdoc.rst\> Forward Command Controller \<../forward_command_controller/doc/userdoc.rst\> Gripper Controller \<../gripper_controllers/doc/userdoc.rst\> Joint Trajectory Controller \<../joint_trajectory_controller/doc/userdoc.rst\> PID Controller \<../pid_controller/doc/userdoc.rst\> Position Controllers \<../position_controllers/doc/userdoc.rst\> Velocity Controllers \<../velocity_controllers/doc/userdoc.rst\>

> 准入控制器/admittance_controller/doc/userdoc.rst\>Effort Controllers\</effort_controllers/doc/userdoc.rst\>正向命令控制器\</forward_command_controller/doc/userdoc.rst\>夹具控制器\</gripper_controllers/doc/userdoc.rst\>联合轨迹控制器\</joint_trajectory_controller/doc/userdoc.rst\>PID 控制器\</pid_controller/doc/userdoc.rst\>位置控制器\</position_controllers/doc/userdoc.rst\>速度控制器\</velocity_controllers/doc/userdoc.rst\>
> :::

## Broadcasters

Broadcasters are used to publish sensor data from hardware components to ROS topics. In the sense of ros2_control, broadcasters are still controllers using the same controller interface as the other controllers above.

> 广播公司用于将硬件组件的传感器数据发布到 ROS 主题。在 ros2_control 的意义上，广播公司仍然是使用与上述其他控制器相同的控制器接口的控制器。

::: {.toctree titlesonly=""}

Force Torque Sensor Broadcaster \<../force_torque_sensor_broadcaster/doc/userdoc.rst\> IMU Sensor Broadcaster \<../imu_sensor_broadcaster/doc/userdoc.rst\> Joint State Broadcaster \<../joint_state_broadcaster/doc/userdoc.rst\> Range Sensor Broadcaster \<../range_sensor_broadcaster/doc/userdoc.rst\>

> 力扭矩传感器广播器/force_torque_sensor_broadcaster/doc/userdoc.rst\>IMU 传感器广播器\</imu_sensor_broadcaster/doc/userdoc.rst\>联合国家广播公司\</joint_state_broadcaster/doc/userdoc.rst\>距离传感器广播器\</range_sensor_broadcaster/doc/userdoc.rst\>
> :::

## Common Controller Parameters

Every controller and broadcaster has a few common parameters. They are optional, but if needed they have to be set before `onConfigure` transition to `inactive` state, see [lifecycle documents](https://design.ros2.org/articles/node_lifecycle.html). Once the controllers are already loaded, this transition is done using the service `configure_controller` of the controller_manager.

> 每个控制器和广播器都有一些通用参数。它们是可选的，但如果需要，必须在“onConfigure”转换为“inactive”状态之前进行设置，请参阅[生命周期文档](https://design.ros2.org/articles/node_lifecycle.html). 一旦控制器已经加载，就使用 controller_manager 的服务“config_controller”来完成此转换。

- `update_rate`: An unsigned integer parameter representing the rate at which every controller/broadcaster runs its update cycle. When unspecified, they run at the same frequency as the controller_manager.

> -“update_rate”：一个无符号整数参数，表示每个控制器/广播器运行其更新周期的速率。如果未指定，则它们以与 controller_manager 相同的频率运行。

- `is_async`: A boolean parameter that is needed to specify if the controller update needs to run asynchronously.

> -“is_async”：一个布尔参数，用于指定控制器更新是否需要异步运行。
