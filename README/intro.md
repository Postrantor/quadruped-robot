---
date: 2024-08-04 10:40:46
---

## 怎么看代码, 从哪里开始看

### 工程结构

结构: 工程 -> 功能包 -> cmakelists.txt
代码: cmakelists.txt -> src/main.cpp ->src/\*.cpp

#### ROS2 编译**工程**结构

```log
> tree -L 1
.
├── build 编译中间产物
├── **install** 运行产物: so / bin
├── log debug
└── **src** 开发

4 directories, 0 files
```

#### **功能包**: cmakelists.txt 解析

```c
# 1. 包
cmake_minimum_required(VERSION 3.5)
project(gazebo_msgs)

# 2. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 3. src/
set(msg_files
  "msg/ContactState.msg"
  "msg/ContactsState.msg"
)

# 4. build/
# so
add_library(${PROJECT_NAME}
  ${source}
)
# bin
add_executable(${PROJECT_NAME}
  main.cpp
  ${source}
)

# 5. install/
install(
  FILES gazebo_msgs_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)
```

#### **src/**: quadruped **工程**结构

划分功能包依据 package.xml

##### src/

```markdown
> tree -L 1
> .
> ├── gazebo_ros_pkgs (gazebo)

├── model (**quadruped**)
├── ros2_control (**controller_manager**)

├── ros2_control_demos (demo)
└── ros2_controllers (demo)

5 directories, 0 files
```

##### src/model

```log
> tree -L 1
.
├── controller
├── env
├── examples
├── format_code.sh
├── interfaces
├── quadruped
├── README
├── README.md
├── simulation
├── thirdpart
└── unitree_sdk

9 directories, 2 files
```

##### src/model/

程序入口(**./simulation/robot_description/launch/launch.py**):

1. **launch.py**(simulation/robot_description) -> **bin**: `example` -> **so**: `quadruped` -> **so**: `unitree_sdk`
2. **launch.py**(simulation/robot_description) -> **bin**: controller_manager(ros2_control) -> **so**: `controllers`
3. **launch.py**(simulation/robot_description) -> **bin**: gazebo -> **config**: `robot_description/config/*.xacro` -> ...

```log
> tree -L 2
.
├── controller/**library**/通过launch加载, controller_manager(bin)->controller(so)
│   ├── joint_state_publisher
│   ├── robot_state_publisher
│   └── unitree_joint_controller
├── env
│   ├── bashrc
│   ├── humble_path
│   └── robot
├── examples/**main.cpp**/通过launch加载, examples(bin)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   ├── real
│   └── simulation
├── format_code.sh
├── interfaces/**library**/被其他包运行时依赖
│   ├── unitree_api
│   ├── unitree_go
│   ├── unitree_msgs
│   └── xpp_msgs
├── quadruped/**library**, examples(bin)->quadruped(so)
│   ├── CMakeLists.txt
│   ├── config
│   ├── docs
│   ├── include
│   ├── package.xml
│   ├── README
│   ├── README.md
│   └── src
├── README
│   ├── bridge_with_ros2.md
│   ├── example_a1_sim.md
│   ├── GAZEBO.md
│   ├── intro.md
│   ├── ros2与gazebo通信.md
│   ├── run_gazebo_with_controller.md
│   └── xarco.md
├── README.md
├── simulation/**launch**, launch.py, config/*.yaml
│   ├── README.md
│   ├── robot_description
│   └── xpp
├── thirdpart/**library**, quadruped(so)->thirdpart(so)
│   ├── amd
│   ├── deeprobotics_legged_sdk
│   ├── lcm
│   ├── matplotlib
│   ├── qpOASES
│   ├── quadprog
│   ├── tiny_ekf
│   ├── tinynurbs
│   └── unitree_legged_sdk/控制电机 sdk(so)
└── unitree_sdk/控制电机 sdk(so)
    ├── CMakeLists.txt
    ├── include
    ├── lib
    ├── package.xml
    ├── README.md
    └── src

37 directories, 22 files
```

## ros2_control

![ros2_control](https://wiki.ros.org/ros_control?action=AttachFile&do=get&target=gazebo_ros_control.png)

##
