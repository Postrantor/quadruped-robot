---
tip: translate by baidu@2024-01-15 05:49:32
---

- [0. command](#0-command)
- [1. Overview](#1-overview)
- [2. Source Code Structure](#2-source-code-structure)
- [3. Build the Project](#3-build-the-project)
  - [Step 1: Install the third party dependencies](#step-1-install-the-third-party-dependencies)
  - [Step 2: Compile the codes](#step-2-compile-the-codes)
- [4. Run the Project in Gazebo Simulator](#4-run-the-project-in-gazebo-simulator)
  - [Step 2: Start the Gazebo simulator and load a robot](#step-2-start-the-gazebo-simulator-and-load-a-robot)
  - [Step 3: Run an example](#step-3-run-an-example)
- [5. Control A Robot to Move Around using Keyboard](#5-control-a-robot-to-move-around-using-keyboard)
  - [Step 2: Control robot by keyboard](#step-2-control-robot-by-keyboard)
- [6. Control A Robot to Move Around using Joystick](#6-control-a-robot-to-move-around-using-joystick)
- [7. Run on Real Robot](#7-run-on-real-robot)

# 0. command

```bash
roslaunch qr_gazebo gazebo_startup.launch wname:=earth
roslaunch qr_gazebo model_spawn.launch rname:=a1 use_xacro:=true use_camera:=false
rosrun examples example_a1_sim
rosrun examples example_keyboard
  # L --> J
```

# 1. Overview

This project branch integrates some advanced algrithms such as MPC and WBC to control quadruped robots. This branch have been tested in **ROS Noetic** under **Ubuntu 20.04**. Some users report it can run in **ROS Melodic** under **Ubuntu 18.04**. To ensure optimal performance, we recommend testing this branch in **ROS Noetic** under **Ubuntu 20.04**. Note that this branch is different from the one in the `main` branch. The architecture and major modules are shown below The codes are tested for Unitree A1 and DeepRobotics Lite3. To fine-tune the MPC and WBC algorithms, you can adjust the corresponding parameters (e.g. KP and KD in **quadruped/src/controllers/wbc/task_set** or weights in WBC locomotion controller).
![](./img/mpc-wbc process diagram.png)

> 该项目分支集成了一些先进的算法，如 MPC 和 WBC 来控制四足机器人。此分支已在 `Ubuntu 20.04` 下的 `ROS Noetic` 中进行了测试。
> 这些代码针对 `Unitree A1` 和 `DeepRobotics Lite3` 进行了测试。要微调 MPC 和 WBC 算法，您可以调整相应的参数(例如 `quadruped/src/controllers/WBC/task_set` 中的 `KP` 和 `KD` 或 `WBC` 运动控制器中的权重)。

# 2. Source Code Structure

The source code includes five major directories

- **examples** contains a few examples to help users understand the usage and the project architecture.
- **quadruped** contains the major modules defining robots, controllers, planner, dynamics and supporting algorithms.
- **robot_description** contains the files needed to represent robot models.
- **simulation** contains the necessary environment to run examples in the simulator.
- **xpp** contains a visualization tool for robots.

> - `examples` 包含一些示例，帮助用户了解使用方法和项目架构；
> - `quadruped` 包含定义机器人、控制器、规划器、动力学和支持算法的主要模块；
> - `robot_description` 包含表示机器人模型所需的文件；
> - `simulation` 包含在模拟器中运行示例所需的环境；
> - `xpp` 包含一个机器人可视化工具；

# 3. Build the Project

## Step 1: Install the third party dependencies

Please ensure that all required dependencies have been installed before building the project

```bash
apt install libyaml-cpp-dev libeigen3-dev liblcm-dev libglm-dev
```

## Step 2: Compile the codes

Navigate to your workspace, build the project using ROS tool `catkin_make`

```bash
cd ${your_workspace}
catkin_make
```

If you have a less powerful machine, there are a few steps you can take to optimize the project performance.

> 如果你有一台功能较弱的机器，你可以采取一些步骤来优化项目性能。

- To improve simulation performance, navigate to the `.world` file (the default file is `earth.world`) located in the **simulation/qr_gazebo/worlds** folder and adjust the `<real_time_update_rate>`. The default value is 1000, but you can reduce it to 800 or lower. Observe the `<real_time_factor>` during simulation, if it falls below 1.0, you can further adjust the `<real_time_factor>` by reducing it to 0.9 or lower.

> - 要提高模拟性能，请导航到 `simulation/qr_gazebo/worlds` 文件夹中的 `.world` 文件(默认文件为 `earth.world`)，并调整 `<real_time_update_rate>`。默认值为 1000，但可以将其减少到 800 或更低。在模拟过程中观察 `<real_time_factor>`，如果它低于 1.0，您可以通过将其降低到 0.9 或更低来进一步调整 `<real_time_factor>`。

- Navigate to the "quadruped" folder, uncomment any `BLAS` related content in the "CMakeLists.txt" file if they are commented and then set the followihng option to `ON`, and ensure `libopenblas-dev` has been installed.

> - 导航到 `quadruped` 文件夹，取消注释 `CMakeLists.txt` 文件中任何与 `BLAS` 相关的内容(如果有注释)，然后将 following 选项设置为 `ON`，并确保已安装 `libopenblas-dev`。

```cmake
option(USE_BLAS "USE MKL BLAS" ON)
```

- To boost the performance of a program running on an Intel chip, consider using the MKL library. To install MKL, download the Intel oneAPI Math Kernel Library Offline Installer. After installation, execute the following command prior to running `catkin_make` on your workspace.

> - 要提高在英特尔芯片上运行的程序的性能，请考虑使用 MKL 库。要安装 MKL，请下载“英特尔 oneAPI 数学内核库脱机安装程序”。安装后，在工作区上运行`catkin_make`之前，请执行以下命令。

```bash
source /opt/intel/oneapi/setvars.sh
```

# 4. Run the Project in Gazebo Simulator

## Step 2: Start the Gazebo simulator and load a robot

```bash
source ${your_workspace}/devel/setup.bash
roslaunch qr_gazebo gazebo_startup.launch wname:=earth
```

The `wname` (optional) parameter specifies the world to be loaded in Gazebo. The default value is `earth`. In a new terminal, spawn a robot model and manage controllers in the simulation environment. Below is for loading a Unitree A1 robot.

> `wname`(可选)参数指定要在 Gazebo 中加载的世界。默认值为`earth`。
> 在一个新的终端中，生成一个机器人模型，并在模拟环境中管理控制器。

```bash
roslaunch qr_gazebo model_spawn.launch rname:=a1 use_xacro:=true use_camera:=false
roslaunch qr_gazebo model_spawn.launch rname:=lite3 use_xacro:=true use_camera:=false
```

- The `rname` (optional) parameter specifies the robot to be loaded in Gazebo. The default value is `a1`.
- The `use_xacro` (optional) parameter determines whether to load the model by `xacro` or `urdf`. The default value is `true`.
- The `use_camera` (optional) parameter controls whether to load camera model with the robot. The default value is `false`.

> - `rname`(可选)参数指定要加载到 Gazebo 中的机器人。默认值为`a1`。
> - `use_xacro`(可选)参数决定是通过`xacro`还是`urdf`加载模型。默认值为`true`。
> - `use_camera`(可选)参数控制是否用机器人加载相机模型。默认值为`false`。

**Note that, after executing the command above, press the `Enter` key to start a robot’s controller. The low-level controller is now active and ready to support the movement of a quadruped robot.**

> **请注意，在执行上述命令后，按下“Enter”键启动机器人的控制器。低级控制器现在处于活动状态，并准备支持四足机器人的运动**

## Step 3: Run an example

Here is an example to control Unitree robot A1 (or DeepRobotics Lite3) to move around. Please check the `user_parameters.yaml` file in the **quadruped/config** folder is properly configured for the robot.
If you want to switch from robot A1 to Lite3, modify the parameters in the `user_parameters.yaml` file and ensure they are set for the new robot. Then execute

> 这里是一个控制 Unitree 机器人 A1(或 DeepRobotics Lite3)四处移动的例子。请检查`quadruped/config`文件夹中的`user_parameters.yaml`文件是否已为机器人正确配置。
> 如果要从 robot A1 切换到 Lite3，请修改`user_parameters.yaml`文件中的参数，并确保为新 robot 设置了这些参数。然后执行

```bash
rosrun examples example_a1_sim
rosrun examples example_lite3_sim
```

# 5. Control A Robot to Move Around using Keyboard

## Step 2: Control robot by keyboard

```bash
rosrun examples example_keyboard
# L --> J
```

`K` : switch control mode
`J` : change the gait
`W` `A` `S` `D` : move forward, left, backward, right
`Q` `E` : rotate left, right
`L` : stop troting
`I` : sit down and exit

# 6. Control A Robot to Move Around using Joystick

Utilizing the ROS joy package, you can operate a quadruped robot by using a game joystick (e.g., Logitech F710) to send speed commands. Once you launch the joy node, activate joy-control mode by pressing the `A` button, then maneuver the joystick handle to control the robot to move.

> 利用 ROS joy，您可以通过使用游戏操纵杆(例如 Logitech F710)发送速度命令来操作四足机器人。
> 启动 joy 节点后，按下 `A` 按钮激活 joy-control 模式，然后操纵操纵杆手柄控制机器人移动。

```bash
apt install ros-${your_ros_version}-joy
rosrun joy joy_node
```

Control the quadruped robot using joystick handle.

`A` : switch joy-control mode
`X` : change the gait

Joystick : control robot's movement

`B` : stop troting
`Y` : sit down and exit

# 7. Run on Real Robot

Note that, if your code is running on an ARM architecture board (e.g. DeepRobotics Lite3), please navigate to the **quadruped** folder and add the following commands in the "CMakeLists.txt" file.

> 请注意，如果您的代码在 ARM 架构板上运行(例如 DeepRobotics Lite3)，请导航到 `quadruped` 文件夹，并在 `CMakeLists.txt` 文件中添加以下命令。

```cmake
set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
```

Launch a ROS master node. To run an example code on a real robot (A1), open a new terminal and execute

```bash
rosrun examples example_a1_real
# or
rosrun examples example_lite3_real
```
