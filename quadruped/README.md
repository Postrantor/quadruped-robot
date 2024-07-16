---
tip: translate by google@2024-05-30 00:17:46
---

The program has been tested on Ubuntu18(gazebo9) and Ubuntu20(gazebo11). Make sure that you computer computation resource is enough. The cost time of running a while loop in the main function(e.g., in `exec/test.cpp`) indicates the information. In gazebo simulation, if the cost time is smaller than 1 ms, the controller is able to run in realtime. If it is larger than 1 ms but smaller than 3ms, the controller runs successfully under ros time, which is slowed down. For example, to keep the controller making action with frequency at 500 Hz(in simulation time measurement), while the loop cost time (in realworld time measurement) is 2.5 ms, you need to set the `real_time_update_rate` in `.world` configuration file of gazebo to be `1000/2.5 = 400`, and make `useRosTime` true.

> 该程序已在 Ubuntu18(gazebo9) 和 Ubuntu20(gazebo11) 上进行过测试。请确保计算机有足够的计算资源。主函数（例如在 `exec/test.cpp`）中运行 while 循环的耗时显示了相关信息。在 gazebo 仿真中，如果代价时间小于 1 毫秒，控制器就能实时运行。如果成本时间大于 1 毫秒但小于 3 毫秒，则控制器在实时时间下运行成功，但运行速度会减慢。例如，若要保持控制器以 500 Hz 的频率（在模拟时间测量中）执行操作，而循环成本时间（在真实世界时间测量中）为 2.5 ms，则需要将 gazebo 配置文件 `.world` 中的 `real_time_update_rate` 设为 `1000/2.5=400`，并将 `useRosTime` 设为 true。

## Step 1: install the third party dependencies:

- Eigen3
- yaml-cpp
- lcm
- Quadprogpp
- ros

```bash
apt install \
  libyaml-cpp-dev \
  libeigen3-dev \
  libglm-dev

# git clone https://gitee.com/zhulinsen1/robots.git ./robots
cd ${ROBOTS_DIR}/src/ascend-quadruped-cpp/third_party/lcm-1.4.0
mkdir build && cd build
cmake .. && make && sudo make install
sudo ldconfig

# install ros, related ros packages and gazebo
# After that, to install ros-gazebo packages, run the following command
apt install ros-{version_name}-catkin ros-{version_name}-gazebo*
```

## Step2: edit compile option.

open the `CMakeLists.txt` file in `ascend-quadruped-cpp` folder, check the compile options so as to match the hardware and applications. In particular, when compiling unitree_legged_skd, the binary `.so` file should be `*amd64.so` in ./lib/ subdirectory, according to current X86/AMD64 platform. If you do not want to compile test examples, please disable `ENABLE_TEST`.

> 打开 `ascend-quadruped-cpp` 文件夹中的 `CMakeLists.txt` 文件，检查编译选项以匹配硬件和应用程序。特别是在编译 unitree_legged_skd 时，根据当前的 X86/AMD64 平台，二进制`.so`文件应为 ./lib/ 子目录下的`*amd64.so`。如果不想编译测试示例，请禁用 `ENABLE_TEST`。

## Step 3: compile the exectuable locomotion controller or other ros packages

```bash
cd ${ROBOTS_DIR}/src/ascend-quadruped-cpp/
mkdir build && cd build
cmake .. && make
```

or in ROS env (recommended),

```bash
cd ${ROBOTS_DIR}
catkin_make
```

Currently, the locomotion controller support Velocity Mode(normal trot), Position Mode(for walk through gaps), Walk Mode(static walk) and Advanced-trot Mode(for climb up stairs), among which you can browse through `src/ascend-quadruped-cpp/config` directory to select corresponding configuration. There are several terrains, such as slope of 15 degree, slope of 30 degree, stairs of 10cm height. You can chose each of them by modifying the value of variable `wname` in `qr_gazebo/launch/normal.launch`. To load the slope terrain, you need to append substring `${ROBOT_DIR}/src/simulation/qr_gazebo/worlds/building_editor_models` to the env variable `GAZEBO_MODEL_PATH`.

> 目前，运动控制器支持
>
> - Velocity Mode（普通小跑）
> - Position Mode（穿过缝隙）
> - Walk Mode（静态行走）
> - Advanced-trot Mode（爬楼梯）
>
> 您可以浏览 `src/ascend-quadruped-cpp/config` 目录选择相应的配置。
>
> 有几种地形，如：
>
> - 15 度斜坡
> - 30 度斜坡
> - 10 厘米高的楼梯
>
> 您可以通过修改 `qr_gazebo/launch/normal.launch` 中变量 `wname` 的值来选择每种地形。要加载斜坡地形，您需要将子串 `${ROBOT_DIR}/src/simulation/qr_gazebo/worlds/building_editor_models` 添加到环境变量 `GAZEBO_MODEL_PATH` 中。

In a word, the table1 list gaits for different terrains.

| gait          | locomotion mode | stance time(s) | duty factor |                      terrian |
| ------------- | :-------------: | :------------: | :---------: | ---------------------------: |
| trot          |    velocity     |      0.3       |     0.6     |         flat ground, slope15 |
| trot          |    position     |      0.3       |     0.6     |                   plum piles |
| walk          |    position     |      7.5       |    0.75     | flat ground, slope15, stairs |
| advanced trot |    velocity     |      0.5       |     0.7     | flat ground, slope15, stairs |

## Step 4: run the controller

You need not to login into subdirectories to launch exec file, instead input the commands to launch gazebo simulation env and to spawn model and manage controllers in the simulation env.

> 您无需登录子目录来启动执行文件，只需输入命令即可启动 gazebo 模拟环境。

```bash
roslaunch qr_gazebo gazebo_startup.launch
roslaunch qr_gazebo model_spawn.launch
```

Then in a new terminal, launch the controller node,

```bash
rosrun ascend_quadruped_cpp sim
```

## Step 5 (Option): Advances

- Joy Control

  With ROS joy package, we can use gamepad (such as Logitech F710) to send speed command to control quadruped. Press Key `A` to switch to joy-control mode after open JOY Node. Then manipulate handles to control the quadruped.

> 通过 ROS joy 软件包，我们可以使用游戏手柄（如罗技 F710）发送速度指令来控制四足动物。打开 JOY 节点后，按 `A` 键切换到 joy 控制模式。然后操纵手柄控制四足动物。

- Visualization

  `xpp` is a ROS package for legged roobt visualization, that wraping Rviz internally. We adopt it for unitree A1 robot. To start it, run following command in terminal:

  ```bash
  roslaunch xpp_examples a1_bag.launch
  ```

- Realsense Camera

  In `model_spawn.launch` file, you can determines wheather to use camera or not, by setting `use_camera` param true or false.

> 在 `model_spawn.launch` 文件中，可以通过设置 `use_camera` 参数 true 或 false 来决定是否使用摄像头。

If you have any problems about this repository, pls contact with Yijie Zhu(zhuyijie2@hisilicon.com).
