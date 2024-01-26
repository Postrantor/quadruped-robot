---
tip: translate by baidu@2024-01-26 20:02:43
---

# Introduction

This package can send control command to real robot from ROS2. You can do low-level control(namely control all joints on robot) and high-level control(namely control the walking direction and speed of robot).

> 该软件包可以从 ROS2 向真实的机器人发送控制命令。可以进行低级控制(即控制机器人上的所有关节)和高级控制(即机器人的行走方向和速度)。

This version is suitable for `unitree_legged_sdk` v3.5.1, namely Go1 robot.

## Packages:

Basic message function: `unitree_legged_msgs`

The interface between ROS and real robot: `unitree_legged_real`

## Environment

We recommand users to run this package in Ubuntu 18.04 and ROS2 eloquent environment

> 我们建议用户在 Ubuntu 18.04 和 ROS2 环境中运行此软件包

# Dependencies

- [unitree_legged_sdk](https://github.com/unitreerobotics): v3.5.1

# Configuration

First, creat a directory.

```sh
mkdir -p ~/ros2_ws/src
```

Then download this package into this `~/ros_ws/src` folder.

After you download this package into this folder, your folder should be like this

> 将此软件包下载到此文件夹后，您的文件夹应该是这样的

```
~/ros2_ws/src/unitree_ros2_to_real
```

And now download unitree_legged_sdk v3.5.1 into the path `~/ros_ws/src/unitree_ros2_to_real`

> 现在将 unitree_legged_sdk v3.5.1 下载到路径`~/ros_ws/src/unitree_ros2_to_real`

# Build

```
colcon build
```

# Setup the net connection

First, please connect the network cable between your PC and robot. Then run `ifconfig` in a terminal, you will find your port name. For example, `enx000ec6612921`.

> 首先，请连接电脑和机器人之间的网络电缆。然后在终端中运行“ifconfig”，您会找到您的端口名。例如，`en0002ec6612921'。

Then, open the `ipconfig.sh` file under the folder `unitree_legged_real`, modify the port name to your own. And run the following commands:

> 然后，打开文件夹`unitree_legged_real`下的`ipconfig.sh`文件，将端口名修改为自己的端口名。并运行以下命令：

```
sudo chmod +x ipconfig.sh
sudo ./ipconfig.sh
```

If you run the `ifconfig` again, you will find that port has `inet` and `netmask` now.

> 如果您再次运行“ifconfig”，您会发现端口现在有“inet”和“netmask”。
> In order to set your port automatically, you can modify `interfaces`:

```
sudo gedit /etc/network/interfaces
```

And add the following 4 lines at the end:

```
auto enx000ec6612921
iface enx000ec6612921 inet static
address 192.168.123.162
netmask 255.255.255.0
```

Where the port name have to be changed to your own.

# Run the package

Before you do high level or low level control, you should run the `ros2_udp` node, which is a bridge that connects users and robot

> 在进行高级或低级控制之前，您应该运行“ros2_udp”节点，它是连接用户和机器人的桥梁

```
ros2 run unitree_legged_real ros2_udp highlevel
```

or

```
ros2 run unitree_legged_real ros2_udp lowlevel
```

it depends which control mode(low level or high level) you want to use.

In the high level mode, you can run the node `ros2_walk_example`

```
ros2 run unitree_legged_real ros2_walk_example
```

In the low level mode, you can run the node `ros2_position_example`

```
ros2 run unitree_legged_real ros2_position_example
```

And before you do the low-level control, please press L2+A to sit the robot down and then press L1+L2+start to make the robot into

> 在进行低级控制之前，请按 L2+A 使机器人坐下，然后按 L1+L2+启动使机器人进入

mode in which you can do joint-level control, finally make sure you hang the robot up before you run low-level control.

> 在这种模式下，你可以进行关节水平控制，最后确保在运行低水平控制之前挂上机器人。
