---
tip: translate by google@2024-05-27 20:55:36
---

# Introduction

Here are the ROS simulation packages of Unitree robots, You can load robots and joint controllers in Gazebo, so you can do low-level control(control the torque, position and angular velocity) on the robot joints. Please watch out that the Gazebo simulation cannot do high-level control, namely walking. Besides of these simulation functions, you can also control your real robots in ROS by the [unitree_ros_to_real](https://github.com/unitreerobotics) packages. For real robots, you can do high-level and low-level control by our ROS packages.

> 这是 Unitree 机器人的 ROS 模拟包，您可以在 Gazebo 中加载机器人和关节控制器，因此您可以在机器人接头上进行低级控制(控制扭矩，位置和角速度)。请注意，Gazebo 模拟不能进行高级控制，即行走。
> 除了这些仿真功能的功能外，您还可以通过[unitree_ros_to_real](https://github.com/unitreerobotics)软件包在 ROS 中控制您的真实机器人。对于真正的机器人，您可以通过 ROS 包装进行高级和低级控制。

## Packages:

- robot description: `go1_description`, `a1_description`, `aliengo_description`, `laikago_description`
- robot and joints controller: `unitree_controller`
- simulation related: `qr_gazebo`, `unitree_legged_control`

# Dependencies

- [ROS](https://www.ros.org/) melodic or ROS kinetic(has not been tested)
- [Gazebo8](http://gazebosim.org/)
- [unitree_legged_msgs](https://github.com/unitreerobotics/unitree_ros_to_real): `unitree_legged_msgs` is a package under [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real).

# Build

If you would like to fully compile the `unitree_ros`, please run the following command to install relative packages.

```bash
# If your ROS is melodic:
apt install \
    ros-<version>-controller-interface \
    ros-<version>-gazebo-ros-control \
    ros-<version>-joint-state-controller \
    ros-<version>-effort-controllers \
    ros-<version>-joint-trajectory-controller
```

And open the file `qr_gazebo/worlds/stairs.world`. At the end of the file:

> 并打开文件`qr_gazebo/worlds/strate.world`。在文件的末尾：

```xml
<include>
    <uri>model:///home/unitree/catkin_ws/src/unitree_ros/qr_gazebo/worlds/building_editor_models/stairs</uri>
</include>
```

Please change the path of `building_editor_models/stairs` to the real path on your PC.

> 请将`building_editor_models/Stairs`的路径更改为 PC 上的真实路径。

Then you can use catkin_make to build:

```bash
cd ~/catkin_ws
catkin_make
```

If you face a dependency problem, you can just run `catkin_make` again.

# Detail of Packages

## unitree_legged_control:

It contains the joints controllers for Gazebo simulation, which allows users to control joints with position, velocity and torque. Refer to "unitree_ros/unitree_controller/src/servo.cpp" for joint control examples in different modes.

> 它包含用于 Gazebo 模拟的关节控制器，该控制器允许用户控制以位置，速度和扭矩的关节。有关不同模式中的联合控制示例，请参阅 `unitree_ros/unitree_controller/src/servo.cpp`。

## The description of robots:

Namely the description of Go1, A1, Aliengo and Laikago. Each package include mesh, urdf and xacro files of robot. Take Laikago as an example, you can check the model in Rviz by:

> 即 GO1，A1，Aliengo 和 Laikago 的描述。每个软件包包括机器人的 mesh, urdf 以及 xacro 文件。以 Laikago 为例，您可以通过以下方式检查 RVIZ 的模型

```bash
roslaunch laikago_description laikago_rviz.launch
```

## qr_gazebo & unitree_controller:

You can launch the Gazebo simulation by the following command:

```bash
roslaunch qr_gazebo normal.launch rname:=a1 wname:=stairs
```

Where the `rname` means robot name, which can be `laikago`, `aliengo`, `a1` or `go1`. The `wname` means world name, which can be `earth`, `space` or `stairs`. And the default value of `rname` is `laikago`, while the default value of `wname` is `earth`. In Gazebo, the robot should be lying on the ground with joints not activated.

> - `rname`含义机器人名称，可以是 `laikago`，`aliengo`，`a1` 或 `go1`;
> - `wname` 表示世界名称，可以是 `earth`，`space`或`stairs`;
> - `rname`的默认值是`laikago`，而`wname`的默认值是`earth`;
>   在 Gazebo 中，机器人应躺在地面上，没有激活关节;

### Stand controller

After launching the gazebo simulation, you can start to control the robot:

> 启动 Gazebo 模拟后，您可以开始控制机器人：

```bash
rosrun unitree_controller unitree_servo
# and you can add external disturbances, like a push or a kick:
rosrun unitree_controller unitree_external_force
```

### Position and pose publisher

Here we showed how to control the position and pose of robot without a controller, which should be useful in SLAM or visual development.

> 在这里，我们展示了如何在没有控制器的情况下控制机器人的位置和姿势，该机器人应该在猛击或视觉发展中有用。

Then run the position and pose publisher in another terminal:

```bash
rosrun unitree_controller unitree_move_kinetic
```

The robot will turn around the origin, which is the movement under the world coordinate. And inside of the source file `move_publisher`, we also offered the method to move robot under robot coordinate. You can change the value of `def_frame` to `coord::ROBOT` and run the `catkin_make` again, then the `unitree_move_publisher` will move robot under its own coordinate.

> 机器人将扭转起源，这是世界下的运动;
> 在源文件 `move_publisher` 的内部，我们还提供了在机器人坐标下移动机器人的方法。
> 您可以将`def_frame`的值更改为`coord::robot`并再次运行 `catkin_make`，然后 `unitree_move_publisher` 将在其自己的坐标下移动机器人。
