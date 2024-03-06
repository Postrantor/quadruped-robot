---
tip: translate by baidu@2024-02-29 20:47:13
github_url: <https://github.com/ros-controls/gazebo_ros2_control/blob/%7BREPOS_FILE_BRANCH%7D/doc/index.rst>
---

> [!note]
> 这个包的作用的使用 ros2_control + gazebo 实现在仿真的效果，ros2_control 的实现需要 hardware+controller
> 因此，需要有 hardware_interface 来实现和 gazebo 的对接，因此就是写一个 gazebo 接口的 hardware
> 这个包写的通用一些，通过配置文件配置支持的 interface/msg 等，和上层的 controller 进行对接。

# gazebo_ros2_control

This is a ROS 2 package for integrating the _ros2_control_ controller architecture with the [Gazebo Classic](https://classic.gazebosim.org/) simulator.

> 这是一个 ROS 2 包，用于将 _ros2_control_ 控制器架构与[Gazebo Classic]集成(https://classic.gazebosim.org/)模拟器。

This package provides a Gazebo plugin which instantiates a _ros2_control_ controller manager and connects it to a Gazebo model.

> 该包提供了一个 Gazebo 插件，该插件实例化 _ros2_control_ 控制器管理器并将其连接到 Gazebo 模型。

![Cart](./0c8e63a38bd8dff196b53a1ed74f5e55308a93e2.gif)
![DiffBot](./ced519dc35fc420722a7d65558e7b3bce01b0f54.gif)

## Usage

### Modifying or building your own

```sh
cd Docker
docker build -t gazebo_ros2_control .
```

### To run the demo

1.  Using Docker

Docker allows us to run the demo without GUI if we don\'t configure it properly. The following command runs the demo without GUI:

> Docker 允许我们在没有正确配置 GUI 的情况下运行演示。以下命令在没有 GUI 的情况下运行演示：

```sh
docker run -it --rm --name gazebo_ros2_control_demo --net host gazebo_ros2_control ros2 launch gazebo_ros2_control_demos cart_example_position.launch.py gui:=false
```

The in your local machine you can run the Gazebo Classic client:

```sh
gzclient
```

2.  Using Rocker

To run the demo with GUI we are going to use [rocker](https://github.com/osrf/rocker/) which is a tool to run docker images with customized local support injected for things like nvidia support. And user id specific files for cleaner mounting file permissions. You can install this tool with the following [instructions](https://github.com/osrf/rocker/#installation).

> 要使用 GUI 运行演示，我们将使用[rocker](https://github.com/osrf/rocker/)这是一个运行 docker 镜像的工具，为 nvidia 支持等注入了定制的本地支持。和用户 id 特定的文件，以获得更干净的装载文件权限。您可以使用以下[说明]安装此工具(https://github.com/osrf/rocker/#installation).

The following command will launch Gazebo Classic:

```sh
rocker --x11 --nvidia --name gazebo_ros2_control_demo gazebo_ros2_control:latest
```

The following commands allow to move the cart in the rail:

```sh
docker exec -it gazebo_ros2_control_demo bash
source /home/ros2_ws/install/setup.bash
ros2 run gazebo_ros2_control_demos example_position
```

## Add ros2_control tag to a URDF

### Simple setup

To use _ros2_control_ with your robot, you need to add some additional elements to your URDF. You should include the tag `<ros2_control>` to access and control the robot interfaces. We should include

> 要在机器人中使用 _ros2_control_，您需要在 URDF 中添加一些附加元素。您应该包含标签`<ros2_control>`来访问和控制机器人接口。我们应该包括

- a specific `<plugin>` for our robot
- `<joint>` tag including the robot controllers: commands and states.

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="slider_to_cart">
    <command_interface name="effort">
      <param name="min">-1000</param>
      <param name="max">1000</param>
    </command_interface>
    <state_interface name="position">
      <param name="initial_value">1.0</param>
    </state_interface>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

### Using mimic joints in simulation

To use `mimic` joints in _gazebo_ros2_control_ you should define its parameters to your URDF. We should include:

> 要在 _gazebo_ros2_control_ 中使用“模拟”关节，应将其参数定义为 URDF。我们应该包括：

- `<mimic>` tag to the mimicked joint [detailed manual](https://wiki.ros.org/urdf/XML/joint)
- `mimic` and `multiplier` parameters to joint definition in `<ros2_control>` tag

```xml
<joint name="left_finger_joint" type="prismatic">
  <mimic joint="right_finger_joint"/>
  <axis xyz="0 1 0"/>
  <origin xyz="0.0 0.48 1" rpy="0.0 0.0 3.1415926535"/>
  <parent link="base"/>
  <child link="finger_left"/>
  <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
</joint>
```

```xml
<joint name="left_finger_joint">
  <param name="mimic">right_finger_joint</param>
  <param name="multiplier">1</param>
  <command_interface name="position"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
</joint>
```

## Add the gazebo_ros2_control plugin

In addition to the _ros2_control_ tags, a Gazebo plugin needs to be added to your URDF that actually parses the _ros2_control_ tags and loads the appropriate hardware interfaces and controller manager. By default the _gazebo_ros2_control_ plugin is very simple, though it is also extensible via an additional plugin architecture to allow power users to create their own custom robot hardware interfaces between _ros2_control_ and Gazebo Classic.

> 除了 _ros2_control_ 标记之外，还需要向 URDF 添加一个 Gazebo 插件，该插件实际解析 _ros2_control_ 标记并加载适当的硬件接口和控制器管理器。默认情况下，_gazebo_ros2_control_ 插件非常简单，但它也可以通过额外的插件架构进行扩展，允许超级用户在 _ros2_control_ 和 gazebo Classic 之间创建自己的自定义机器人硬件接口。

```xml
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <robot_param>robot_description</robot_param>
    <robot_param_node>robot_state_publisher</robot_param_node>
    <parameters>$(find gazebo_ros2_control_demos)/config/cart_controller.yaml</parameters>
  </plugin>
</gazebo>
```

The _gazebo_ros2_control_ `<plugin>` tag also has the following optional child elements:

> _gazebo_ros2_control_`<plugin>`标记还有以下可选子元素：

- `<robot_param>`: The location of the `robot_description` (URDF) on the parameter server, defaults to `robot_description`
- `<robot_param_node>`: Name of the node where the `robot_param` is located, defaults to `robot_state_publisher`
- `<parameters>`: YAML file with the configuration of the controllers

- `<robot_param>`：参数服务器上`robot_description` (URDF) 的位置，默认为`robot_description`
- `<robot_param_node>`：`robot_param`所在节点的名称，默认为`robot_state_publisher`
- `<parameters>`：包含控制器配置的 YAML 文件

### Default gazebo_ros2_control Behavior

> [!note]
> 某些节点也会将 URDF 作为参数服务器，从中获取数据
> 忘记是从哪里看到的流程图了，是有这样的一个环节

By default, without a `<plugin>` tag, _gazebo_ros2_control_ will attempt to get all of the information it needs to interface with a ros2_control-based controller out of the URDF. This is sufficient for most cases, and good for at least getting started.

> 默认情况下，如果没有`<plugin>`标记，_gazebo_ros2_control_ 将尝试从 URDF 中获取与基于 ros2_control 的控制器接口所需的所有信息。这对大多数情况来说已经足够了，而且至少对入门来说是有益的。

The default behavior provides the following ros2_control interfaces:

- `hardware_interface::JointStateInterface`
- `hardware_interface::EffortJointInterface`
- `hardware_interface::VelocityJointInterface`

### Advanced: custom gazebo_ros2_control Simulation Plugins

The _gazebo_ros2_control_ Gazebo plugin also provides a pluginlib-based interface to implement custom interfaces between Gazebo Classic and _ros2_control_ for simulating more complex mechanisms (nonlinear springs, linkages, etc).

> gazebo_ros2_control 插件还提供了一个基于 pluginlib 的接口，以实现 gazebo Classic 和 ros2_control 之间的自定义接口，用于模拟更复杂的机构(非线性弹簧、连杆等)。

These plugins must inherit `gazebo_ros2_control::GazeboSystemInterface` which implements a simulated _ros2_control_ `hardware_interface::SystemInterface`. `SystemInterface` provides API-level access to read and command joint properties.

> 这些插件必须继承实现模拟 _ros2_control_ `hardware_interface::SystemInterface` 的 `gazebo_ros2_control::GazeboSystemInterface`。`SystemInterface` 提供对读取和命令关节属性的 API 级访问。

The respective `GazeboSystemInterface` sub-class is specified in a URDF model and is loaded when the robot model is loaded. For example, the following XML will load the default plugin:

> 相应的 `GazeboSystemInterface` 子类在 URDF 模型中指定，并在加载机器人模型时加载。例如，以下 XML 将加载默认插件：

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  ...
<ros2_control>
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    ...
  </plugin>
</gazebo>
```

### Set up controllers

Use the tag `<parameters>` inside `<plugin>` to set the YAML file with the controller configuration.

> 使用`<plugin>`内的标签`<parameters>`设置带有控制器配置的 YAML 文件。

```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find gazebo_ros2_control_demos)/config/cart_controller.yaml</parameters>
  </plugin>
<gazebo>
```

The following is a basic configuration of the controllers:

- `joint_state_broadcaster`: This controller publishes the state of all resources registered to a `hardware_interface::StateInterface` to a topic of type `sensor_msgs/msg/JointState`.
- `joint_trajectory_controller`: This controller creates an action called `/joint_trajectory_controller/follow_joint_trajectory` of type `control_msgs::action::FollowJointTrajectory`.

> - `joint_state_broadcaster`：此控制器将注册到 `hardware_interface::StateInterface` 的所有资源的状态发布到类型为 `sensor_msgs/msg/JointState` 的主题。
> - `joint_trajectory_controller`：此控制器创建一个名为 `/joint_trajectory_controller/follow_joint_trajectory` 的操作，其类型为 `control_msgs::action::FollowJointTrajectory`。

::: {.literalinclude language="yaml"}
../gazebo_ros2_control_demos/config/cart_controller.yaml
:::

## gazebo_ros2_control_demos

This package contains the contents for testing `gazebo_ros2_control`. It is running Gazebo Classic and some other ROS 2 nodes. There are some examples in the _Gazebo_ros2_control_demos_ package. These examples allow to launch a cart in a 30 meter rail.

> 此包包含用于测试 `gazebo_ros2_control` 的内容。它正在运行 Gazebo Classic 和其他一些 ROS 2 节点。
> _Gazebo_ros2_control_demos_ 包中有一些示例。这些例子允许在 30 米的轨道上启动推车。

![Cart](./4590c6bd733bfe2d4b5d3e08894f2c39dda883d9.gif)

You can run some of the configuration running the following commands:

```sh
ros2 launch gazebo_ros2_control_demos cart_example_position.launch.py
ros2 launch gazebo_ros2_control_demos cart_example_velocity.launch.py
ros2 launch gazebo_ros2_control_demos cart_example_effort.launch.py
ros2 launch gazebo_ros2_control_demos diff_drive.launch.py
ros2 launch gazebo_ros2_control_demos tricycle_drive.launch.py
```

When the Gazebo world is launched you can run some of the following commands to move the cart.

> 当 Gazebo 世界启动时，您可以运行以下一些命令来移动推车。

```sh
ros2 run gazebo_ros2_control_demos example_position
ros2 run gazebo_ros2_control_demos example_velocity
ros2 run gazebo_ros2_control_demos example_effort
ros2 run gazebo_ros2_control_demos example_diff_drive
ros2 run gazebo_ros2_control_demos example_tricycle_drive
```

The following example shows parallel gripper with mimic joint:

![Cart](./8ee140fe230277b385ad586aea1da5406359f22b.gif)

```sh
ros2 launch gazebo_ros2_control_demos gripper_mimic_joint_example.launch.py
```

Send example commands:

```sh
ros2 run gazebo_ros2_control_demos example_gripper
```

## run

```sh
. /usr/share/gazebo/setup.sh
# ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity diffbot
ros2 launch gazebo_ros2_control_demos diff_drive.launch.py
ros2 run gazebo_ros2_control_demos example_diff_drive
```
