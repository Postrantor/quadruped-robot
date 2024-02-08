---
tip: translate by baidu@2024-01-23 23:16:05
---

Packages Version: v3.8.0

# Introduction

This package can send control command to real robot from ROS. You can do low-level control(namely control all joints on robot) and high-level control(namely control the walking direction and speed of robot).

> 该软件包可以从 ROS 向真实的机器人发送控制命令。可以进行低级控制(即控制机器人上的所有关节)和高级控制(即机器人的行走方向和速度)。

This version is suitable for unitree_legged_sdk v3.5.1, namely Go1 robot.

## Packages:

Basic message function: `unitree_legged_msgs`

The interface between ROS and real robot: `unitree_legged_real`

## Environment

We recommand users to run this package in Ubuntu 18.04 and ROS melodic environment

## Dependencies

- [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk/releases)

### Notice

The newest release [v3.8.0](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/3.8.0) only supports for robot: Go1.

Check release [v3.3.4](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/3.3.4) for A1 support.

# Configuration

Before compiling this package, please download the corresponding unitree_legged_sdk as noted above, and put it to your own workspace's source folder(e.g. `~/catkin_ws/src`). Be careful with the sdk folder name. It should be "unitree_legged_sdk" without version tag.

> 在编译此包之前，请如上所述下载相应的 unitree_legged_sdk，并将其放在您自己工作区的源文件夹中(例如`~/catkin_ws/src`)。注意 sdk 文件夹名称。它应该是“unitree_legged_sdk”，没有版本标记。

# Build

You can use catkin_make to build ROS packages. First copy the package folder to `~/catkin_ws/src`, then:

> 您可以使用 catkin_make 来构建 ROS 包。首先将包文件夹复制到`~/catkin_ws/src`，然后：

```sh
cd ~/catkin_ws
catkin_make
```

# Setup the net connection

First, please connect the network cable between your PC and robot. Then run `ifconfig` in a terminal, you will find your port name. For example, `enx000ec6612921`.

> 首先，请连接电脑和机器人之间的网络电缆。然后在终端中运行“ifconfig”，您会找到您的端口名。例如，`en0002ec6612921'。

Then, open the `ipconfig.sh` file under the folder `unitree_legged_real`, modify the port name to your own. And run the following commands:

> 然后，打开文件夹`unitree_legged_real`下的`ipconfig.sh`文件，将端口名修改为自己的端口名。并运行以下命令：

```sh
sudo chmod +x ipconfig.sh
sudo ./ipconfig.sh
```

If you run the `ifconfig` again, you will find that port has `inet` and `netmask` now.

> 如果您再次运行“ifconfig”，您会发现端口现在有“inet”和“netmask”。

In order to set your port automatically, you can modify `interfaces`:

```sh
sudo gedit /etc/network/interfaces
```

And add the following 4 lines at the end:

```sh
auto enx000ec6612921
iface enx000ec6612921 inet static
address 192.168.123.162
netmask 255.255.255.0
```

Where the port name have to be changed to your own.

# Run the package

You can control your real Go1 robot from ROS by this package.

Before you run expamle program, please run command

```sh
roslaunch unitree_legged_real real.launch ctrl_level:=highlevel
```

or

```sh
roslaunch unitree_legged_real real.launch ctrl_level:=lowlevel
```

It depends which control mode you want to use.

Then, if you want to run high-level control mode, you can run example_walk node like this

> 然后，如果您想运行高级控制模式，可以像这样运行 example_walk 节点

```sh
rosrun unitree_legged_real ros_example_walk
```

If you want to run low-level control mode, you can run example_position program node like this

> 如果你想运行低级控制模式，你可以像这样运行 example_position 程序节点

```sh
rosrun unitree_legged_real ros_example_postion
```

You can also run the node state_sub to subscribe the feedback information from Go1 robot

> 您还可以运行节点 state_sub 来订阅 Go1 机器人的反馈信息

```sh
rosrun unitree_legged_real state_sub
```

You can also run the launch file that enables you control robot via keyboard like you can do in turtlesim package

> 您还可以运行启动文件，使您能够像在 turtlesim 包中一样通过键盘控制机器人

```sh
roslaunch unitree_legged_real keyboard_control.launch
```

And before you do the low-level control, please press L2+A to sit the robot down and then press L1+L2+start to make the robot into

> 在进行低级控制之前，请按 L2+A 使机器人坐下，然后按 L1+L2+启动使机器人进入

mode in which you can do joint-level control, finally make sure you hang the robot up before you run low-level control.

> 在这种模式下，你可以进行关节水平控制，最后确保在运行低水平控制之前挂上机器人。
