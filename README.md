## git

[xpp](https://github.com/leggedrobotics/xpp.git)
[gazebosim/ros_gz_project_template at fortress](https://github.com/gazebosim/ros_gz_project_template/tree/fortress)

## run

在虚拟机中运行 gazebo 会出现一些问题，可能是软件版本相关？
[Gazebo Ignition 无法在 Ubuntu VM 中渲染场景](https://github.com/gazebosim/gz-sim/issues/1492)

需要使用参数：

```sh
ign gazebo lights.sdf -v 4 --render-engine ogre
```

## url

https://gazebosim.org/docs/fortress/ros2_integration
https://github.com/gazebosim/gz-plugin/blob/main/loader/include/gz/plugin/detail/Registry.hh
https://github.com/gazebosim/gz-transport
https://github.com/gazebosim/ros_gz/blob/humble/README.md
https://gazebosim.org/api/transport/9.0/topicstatistics.html
https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html#add-ros2-control-tag-to-a-urdf
https://classic.gazebosim.org/tutorials?tut=custom_messages

## install

gazebo 需要安装和 ROS2 相互匹配的版本，考虑一部分原因是因为两者需要通过 bridge 来实现通信(https://github.com/ignitionrobotics/ros_ign/blob/ros2/ros_gz_bridge/README.md)，应该是有耦合的部分。考虑这部分原因，需要安装相互匹配的版本实现通信，具体需要如下指令通过二进制的方式进行集成。具体而言，在 `humble` 中使用的是 `fortress` 版本。

```sh
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

如果需要使用非官方版本，则需要声明环境变量 `export GZ_VERSION=${gz_version}`，因此 humble 中可以缺省使用 `export GZ_VERSION=fortress`

> [!note]
> 相关文档可以参考 [Gazebo - Docs: ROS/Gazebo Installation](https://gazebosim.org/docs/fortress/ros_installation)
> Humble officially supports Gazebo Fortress

## ros_gz_bridge

该包提供了一个网络桥，可以在 ROS 和 Gazebo Transport 之间交换消息。其中还有“静态类型”的消息通信，可以好好理解一些 bridge 的使用。详细信息可以参见[ros2·gazebosim/ros_gz](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md)

我们可以初始化一个双向桥，这样我们就可以将 ROS 作为发布者，将 Ignition 作为订阅者，反之亦然。

```sh
ros2 run ros_ign_bridge parameter_bridge /TOPIC@ROS_MSG@IGN_MSG
```

文章内有相关的 example 可以参考[gazebosim/ros_gz](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md#example-1a-ignition-transport-talker-and-ros-2-listener)，具体可以尝试 example1a：

```sh
# Shell A:
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@ignition.msgs.StringMsg
# Shell B:
ros2 topic echo /chatter
# Shell C:
ign topic -t /chatter -m ignition.msgs.StringMsg -p 'data:"Hello"'
```

https://github.com/search?q=repo%3Agazebosim%2Fdocs%20path%3A%2F%5Efortress%5C%2F%2F%20Plugins&type=code
https://github.com/gazebosim/docs/blob/f5f994667ee735e2956aec301ae60df9ce6a0578/fortress/GUI_tutorial.md
https://github.com/gazebosim/docs/blob/f5f994667ee735e2956aec301ae60df9ce6a0578/fortress/sdf_worlds.md?plain=1#L35
https://github.com/gazebosim/docs/blob/f5f994667ee735e2956aec301ae60df9ce6a0578/fortress/moving_robot.md?plain=1#L10
https://github.com/gazebosim/docs/blob/f5f994667ee735e2956aec301ae60df9ce6a0578/fortress/Model_insertion_fuel.md?plain=1#L37
https://github.com/gazebosim/docs/blob/f5f994667ee735e2956aec301ae60df9ce6a0578/fortress/sensors.md?plain=1#L160
https://github.com/gazebosim/docs/blob/f5f994667ee735e2956aec301ae60df9ce6a0578/fortress/ros_gz_project_template_guide.md?plain=1#L64
https://gazebosim.org/docs/fortress/ros2_integration
https://classic.gazebosim.org/tutorials?tut=custom_messages
https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html#add-ros2-control-tag-to-a-urdf

## gazebo_ros2_control

该功能包有一些实例可以尝试运行，提供了 gazebo_classic + ros2 + ros2-control 的示例。

> [!note]
>
> 1. 在执行示例的时候，启动 gazebo 有一些问题，对于 python 脚本需要先给出 chmod +x 的权限，即使 install 中的脚本具有权限，也要对 script 中的脚本文件添加可执行权限；
> 2. 对于启动过程中遇到的 gzclient error，可能是由于一些环境变量被覆盖，需要重新 source，详细可以参考 [无法在启动文件上启动 gzclient - 导致共享指针断言错误](https://answers.ros.org/question/358847/cannot-launch-gzclient-on-a-launch-file-results-in-shared_ptr-assertion-error/)
>
> - `. /usr/share/gazebo/setup.sh`
> - `ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity diffbot`
> - `ros2 run gazebo_ros2_control_demos example_diff_drive`
