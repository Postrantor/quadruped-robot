---
tip: translate by baidu@2024-02-12 19:20:45
github_url: |
  <https://github.com/ros-controls/ros2_controllers/blob/%7BREPOS_FILE_BRANCH%7D/diff_drive_controller/doc/userdoc.rst>
---

# diff_drive_controller {#diff_drive_controller_userdoc}

Controller for mobile robots with differential drive.

As input it takes velocity commands for the robot body, which are translated to wheel commands for the differential drive base.

> 作为输入，它接受机器人主体的速度命令，这些命令被转换为差速驱动底座的车轮命令。

Odometry is computed from hardware feedback and published.

## Other features

> - Realtime-safe implementation.
> - Odometry publishing
> - Task-space velocity, acceleration and jerk limits
> - Automatic stop after command time-out

## Description of controller\'s interfaces

### References

(the controller is not yet implemented as chainable controller)

### Feedback

As feedback interface type the joints\' position (`hardware_interface::HW_IF_POSITION`) or velocity (`hardware_interface::HW_IF_VELOCITY`,if parameter `position_feedback=false`) are used.

> 作为反馈接口类型，如果参数“position_feedback=false”，则使用关节的位置（“hardware_interface:：HW_IF_position”）或速度（“hardware \_interface:HW_IF_velocity”）。

### Output

Joints\' velocity (`hardware_interface::HW_IF_VELOCITY`) are used.

## ROS 2 Interfaces

### Subscribers

\~/cmd_vel \[geometry_msgs/msg/TwistStamped\]

: Velocity command for the controller, if `use_stamped_vel=true`. The controller extracts the x component of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.

> ：控制器的速度命令，如果`use_stamped_vel=true`。控制器提取线速度的 x 分量和角速度的 z 分量。忽略其他组件上的速度。

### Publishers

\~/odom \[nav_msgs::msg::Odometry\]

: This represents an estimate of the robot\'s position and velocity in free space.

> ：这表示对机器人在自由空间中的位置和速度的估计。

/tf \[tf2_msgs::msg::TFMessage\]

: tf tree. Published only if `enable_odom_tf=true`

\~/cmd_vel_out \[geometry_msgs/msg/TwistStamped\]

: Velocity command for the controller, where limits were applied. Published only if `publish_limited_velocity=true`

> ：控制器的速度命令，其中应用了限制。仅当`publish_limited_versity=true时发布`

### Parameters

This controller uses the [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library) to handle its parameters. The parameter [definition file located in the src folder](https://github.com/ros-controls/ros2_controllers/blob/%7BREPOS_FILE_BRANCH%7D/diff_drive_controller/src/diff_drive_controller_parameter.yaml) contains descriptions for all the parameters used by the controller.

> 此控制器使用[generate_parameter_library]以处理其参数。参数[位于 src 文件夹中的定义文件]包含控制器使用的所有参数的描述。

An example parameter file for this controller can be found in [the test directory](https://github.com/ros-controls/ros2_controllers/blob/%7BREPOS_FILE_BRANCH%7D/diff_drive_controller/test/config/test_diff_drive_controller.yaml):

> 可以在[测试目录]中找到此控制器的示例参数文件:

::: {.literalinclude language="yaml"}
../test/config/test_diff_drive_controller.yaml
:::
