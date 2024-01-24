---
tip: translate by baidu@2024-01-24 23:08:25
---

A more detailed unitree_guide document is available at unitree [developer site](https://support.unitree.com/home/zh/Algorithm_Practice).( only Chinese version now )

> 更详细的 unitree_guide 文档可在 unitree[开发者网站]上获得(https://support.unitree.com/home/zh/Algorithm_Practice).（现在只有中文版）

# Overview

The unitree_guide is an open source project for controlling the quadruped robot of Unitree Robotics, and it is also the software project accompanying [《四足机器人控制算法--建模、控制与实践》](https://detail.tmall.com/item.htm?spm=a212k0.12153887.0.0.5487687dBgiovR&id=704510718152) published by Unitree Robotics.

> unitree_guide 是 unitree Robotics 的一个控制四足机器人的开源项目，也是伴随而来的软件项目[《四足机器人控制算法--建模、控制与实践》](https://detail.tmall.com/item.htm?spm=a212k0.12153887.0.0.5487687dBgiovR&id=704510718152)由 Unitree Robotics 出版。

# Quick Start

The following will quickly introduce the use of unitree_guide in the gazebo simulator. For more usage, please refer to 《四足机器人控制算法--建模、控制与实践》.

> 下面将快速介绍 unitree_guide 在凉亭模拟器中的使用。有关更多用法，请参阅《四足机器人控制算法--建模、控制与实践》.

## Environment

We recommand users to run this project in Ubuntu 18.04 and ROS melodic environment.

> 我们建议用户在 Ubuntu 18.04 和 ROS 旋律环境中运行此项目。

## Dependencies

1. [unitree_guide](https://github.com/unitreerobotics/unitree_guide)<br>
2. [unitree_ros](https://github.com/unitreerobotics/unitree_ros)<br>
3. [unitree_legged_msgs](https://github.com/unitreerobotics/unitree_ros_to_real)(Note that: unitree_legged_real package should not be a part of dependencies)<br>

Put these three packages in the src folder of a ROS workspace.

## build

Open a terminal and switch the directory to the ros workspace containing unitree_guide, then run the following command to build the project:

> 打开一个终端，将目录切换到包含 unitree_guide 的 ros 工作区，然后运行以下命令来构建项目：

```
catkin_make
source ./devel/setup.bash
roslaunch unitree_guide gazeboSim.launch
./devel/lib/unitree_guide/junior_ctrl
```

## usage

After starting the controller, the robot will lie on the ground of the simulator, then press the '2' key on the keyboard to switch the robot's finite state machine (FSM) from **Passive**(initial state) to **FixedStand**, then press the '4' key to switch the FSM from **FixedStand** to **Trotting**, now you can press the 'w' 'a' 's' 'd' key to control the translation of the robot, and press the 'j' 'l' key to control the rotation of the robot. Press the Spacebar, the robot will stop and stand on the ground

> 启动控制器后，机器人将躺在模拟器的地面上，然后按下键盘上的“2”键将机器人的有限状态机（FSM）从**被动**（初始状态）切换到**固定支架**，然后按下“4”键将 FSM 从**固定支架**切换到**旋转**，现在可以按下“w”a“s”d“键来控制机器人的平移，并按下“j”“l”键来控制机器人的旋转。按下空格键，机器人将停止并站在地面上

. (If there is no response, you need to click on the terminal opened for starting the controller and then repeat the previous operation)

> 。（如果没有响应，则需要点击打开的终端以启动控制器，然后重复前面的操作）

# Note

Unitree_guide provides a basic quadruped robot controller for beginners. To achive better performance, additional fine tuning of parameters or more advanced methods (such as MPC etc.) might be required. Any contribution and good idea from the robotics community are all welcome. Feel free to raise an issue ~ <br>

> Unitree_guide 为初学者提供了一个基本的四足机器人控制器。为了获得更好的性能，可能需要额外的参数微调或更先进的方法（如 MPC 等）。欢迎机器人界的任何贡献和好主意。欢迎提出问题~<br>
