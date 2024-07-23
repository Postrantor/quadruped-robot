---
tip: translate by baidu@2024-03-09 00:47:37
---

- [Joint State Publisher](#joint-state-publisher)
  - [Published Topics](#published-topics)
  - [Subscribed Topics](#subscribed-topics)
  - [Parameters](#parameters)
    - [Mapped Parameters](#mapped-parameters)

# Joint State Publisher

This contains a package for publishing `sensor_msgs/msg/JointState` messages for a robot described with URDF. Given a URDF (either passed on the command-line or via the `/robot_description` topic), this node will continually publish values for all of the movable joints in the URDF to the `/joint_states` topic. In combination with `robot_state_publisher`, this ensures that there is a valid transform for all joints even when the joint doesn't have encoder data.

> 其中包含一个包，用于为使用 URDF 描述的机器人发布 `sensor_msgs/msg/JointState` 消息。给定一个 URDF(通过命令行或 `/robot_description` 主题传递)，**此节点将不断将 URDF 中所有可移动关节的值发布到 `/joint_states` 主题**。与 `robot_state_publisher` 相结合，即使关节没有编码器数据，也能确保所有关节都有有效的变换。

## Published Topics

- `/joint_states` (`sensor_msgs/msg/JointState`) - The state of all of the movable joints in the system.

> -`/joint_states`(`sensor_msgs/msg/JJointState`)
> 系统中所有可移动关节的状态。

## Subscribed Topics

- (optional) `/robot_description` (`std_msgs/msg/String`) - If no URDF is given on the command-line, then this node will listen on the `/robot_description` topic for the URDF to be published. Once it has been received at least once, this node will start to publish joint values to `/joint_states`.
- (optional) `/any_topic` (`sensor_msgs/msg/JointState`) - If the `sources_list` parameter is not empty (see Parameters below), then every named topic in this parameter will be subscribed to for joint state updates. Do _not_ add the default `/joint_states` topic to this list, as it will end up in an endless loop!

> - (可选) `/robot_description` (`std_msgs/msg/String`)
>   如果命令行上没有给出 URDF，则此节点将侦听要发布的 URDF 的 `/robo_description` 主题。一旦接收到至少一次，该节点将开始将 joint 值发布到 `/joint_states`。
> - (可选) `/any_topic` (`sensor_msgs/msg/JointState`)
>   如果 `sources_list` 参数不为空(请参阅下面的参数)，则将订阅此参数中的每个命名主题以进行 joint 状态更新。不要将默认的`/joint_states`主题添加到此列表中，因为它将以无休止的循环！

## Parameters

- `rate` (int) - The rate at which to publish updates to the `/joint_states` topic. Defaults to 10.
- `publish_default_positions` (bool) - Whether to publish a default position for each movable joint to the `/joint_states` topic. Defaults to True.
- `publish_default_velocities` (bool) - Whether to publish a default velocity for each movable joint to the `/joint_states` topic. Defaults to False.
- `publish_default_efforts` (bool) - Whether to publish a default effort for each movable joint to the `/joint_states` topic. Defaults to False.
- `use_mimic_tags` (bool) - Whether to honor `<mimic>` tags in the URDF. Defaults to True.
- `use_smallest_joint_limits` (bool) - Whether to honor `<safety_controller>` tags in the URDF. Defaults to True.
- `source_list` (array of strings) - Each string in this array represents a topic name. For each string, create a subscription to the named topic of type `sensor_msgs/msg/JointStates`. Publication to that topic will update the joints named in the message. Defaults to an empty array.
- `delta` (double) - How much to automatically move joints during each iteration. Defaults to 0.0.

> - `rate`(int)
>   发布对`/joint_states`主题的更新的速率，默认值为 10;
> - `publish_default_positions`(bool)
>   是否将每个可移动关节的默认位置发布到 `/joint_states` 主题，默认为 True;
> - `publish_default_velocity`(bool)
>   是否将每个可移动关节的默认速度发布到 `/joint_states` 主题，默认为 False;
> - `publish_default_efforts`(bool)
>   是否将每个可移动关节的默认受力发布到 `/joint_states` 主题，默认为 False;
> - `use_smallest_joint_limits`(bool)
>   是否遵守 URDF 中的`<safety_controller>`标记，默认为 True;
> - `source_list`(字符串数组)
>   此数组中的每个字符串表示一个主题名称。对于每个字符串，创建对类型为 `sensor_msgs/msg/JointStates` 的命名主题的订阅。发布到该主题将更新消息中命名的关节，默认为空数组。

> [!note]
> 应该是 `sensor_msgs/msg/JointStates` 类型的数据？
> 在数据类型以及代码中都没有检索到 JointStates

### Mapped Parameters

These parameters map from "joint_names" to "values". The format to use these parameters is `<parameter>.<key>:=<value>`, where a new parameter is defined for each key. See below for examples.

> 这些参数从 "关节名称" 映射到 "值"。使用这些参数的格式为`<parameter>.<key>:=<value>`，其中为每个键定义一个新参数。请参阅以下示例。

- `zeros` (map from string -> float) - A map of joint_names to initial starting values for the joint. For example, in Eloquent and beyond, this parameter can be used from the command-line with `ros2 run joint_state_publisher joint_state_publisher --ros-args --param zeros.joint_name1:=value1 --param zeros.joint_name2:=value2`. This parameter is not set by default, so all joints start at zero. For joints where zero isn't within the joint range, it uses the range's (max + min) / 2.

> - `zeros` (map from string -> float)
>   joint_name 到关节初始起始值的映射。例如，在 Eloquent 及更高版本中，此参数可以从命令行与`ros2 run joint_state_publisher joint_state_publisher --ros-args --param zeros.joint_name1:=value1 --param zeros.joint_name2:=value2`一起使用。
>   默认情况下不会设置此参数，因此所有关节都从零开始。对于零不在关节范围内的关节，它使用该范围的(最大+最小)/2。

- `dependent_joints` (map from string -> map from 'parent', 'factor', 'offset' -> float) - A map of joint_names to the joints that they mimic; compare to the `<mimic>` tag in URDF. A joint listed here will mimic the movements of the 'parent' joint, subject to the 'factor' and 'offset' provided. The 'parent' name must be provided, while the 'factor' and 'offset' parameters are optional (they default to 1.0 and 0.0, respectively). For example, in Eloquent and beyond, this parameter can be used from the command-line with `ros2 run joint_state_publisher joint_state_publisher --ros-args --param dependent_joints.left_leg.parent:=right_leg --param dependent_joints.left_leg.offset:=0.0 --param dependent_joints.left_leg.factor:=2.0`. This parameter is not set by default, in which case only joints that are marked as `<mimic>` in the URDF are mimiced.

> - `dependent_joints` (`parent`、`factor`、`offset` -> float)
>   joint_name 到它们模拟的关节的映射；与 URDF 中的`<mimic>`标记进行比较。此处列出的关节将模仿`父`关节的运动，但须符合所提供的`系数`和`偏移`。必须提供`parent`名称，而`factor`和`offset`参数是可选的(它们分别默认为 1.0 和 0.0)。
>   例如，在 Eloquent 及更高版本中，此参数可以从命令行与 `ros2 run joint_state_publisher joint_state_publisher --ros-args --param dependent_joints.left_leg.parent:=right_leg --param dependent_joints.left_leg.offset:=0.0 --param dependent_joints.left_leg.factor:=2.0` 一起使用。
>   默认情况下不设置此参数，在这种情况下，仅模拟 URDF 中标记为`<mimic>`的关节。
