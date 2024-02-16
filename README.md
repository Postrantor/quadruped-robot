---
url: https://zhuanlan.zhihu.com/p/658524803
title: 【ROS2】ros2-control 应用
date: 2024-02-12 11:32:24
tag:
summary:
---

## 前言

上一篇文章介绍了 ros2-control 的整体框架，本文介绍具体如何使用 ros2-control。ros2-control 框架中，很多逻辑都不需要自行编写，**用户只需要实现一些重要的接口即可**(很多不重要的接口都不用实现)，从而提升了程序的可移植性。主要要写的有三个部分：**参数文件(config/)、控制器(controller)和硬件组件(hardware_interface)**，此外一些 launch 文件的编写当然也是必要的。

- [前言](#前言)
- [1. 编写参数文件](#1-编写参数文件)
  - [控制器相关参数](#控制器相关参数)
  - [硬件相关参数](#硬件相关参数)
- [2. 编写 `Controller`](#2-编写-controller)
  - [创建包和文件](#创建包和文件)
  - [编写头文件](#编写头文件)
  - [编写源文件](#编写源文件)
    - [实现 `on_init()` 方法](#实现-on_init-方法)
    - [实现 `on_configure()` 方法](#实现-on_configure-方法)
    - [实现 状态接口(state\_interface) / 命令接口(command\_interface)](#实现-状态接口state_interface--命令接口command_interface)
    - [实现 `on_activate()` 方法](#实现-on_activate-方法)
    - [实现 `update()` 方法](#实现-update-方法)
    - [导出 pluginlib](#导出-pluginlib)
  - [修改 CMakeLists](#修改-cmakelists)
  - [修改 package.xml](#修改-packagexml)
- [3. 编写 `Hardware`](#3-编写-hardware)
  - [创建包和文件](#创建包和文件-1)
  - [编写头文件](#编写头文件-1)
  - [编写源文件](#编写源文件-1)
    - [编写 `on_init()` 函数](#编写-on_init-函数)
    - [实现 `export_command_interfaces()` 和 `export_state_interfaces()`](#实现-export_command_interfaces-和-export_state_interfaces)
    - [编写 `on_configure()` 函数](#编写-on_configure-函数)
    - [编写 `on_activate()` 函数](#编写-on_activate-函数)
    - [编写 `on_deactivate()` 函数](#编写-on_deactivate-函数)
    - [编写 `read()` 和 `write()` 函数](#编写-read-和-write-函数)
    - [导出 `pluginlib`](#导出-pluginlib-1)
  - [修改 CMakeLists](#修改-cmakelists-1)
  - [修改 package.xml](#修改-packagexml-1)
- [4. 小结](#4-小结)

## 1. 编写参数文件

需要实现两个类型的参数：

- **控制器相关参数**：这部分保存在一个 yaml 文件中，一般是与实际控制算法相关的参数；
- **硬件相关参数**：这部分保存在 URDF 中。

在编写实际的包之前，我们再看一遍下面的框图，理解各个模块在 ros2-control 框架中的位置。

![](https://pic1.zhimg.com/v2-0b26a9d2f615663b333d986d64900278_r.jpg)

### 控制器相关参数

这部分参数又包括 `controller_manager` 参数和每个具体的 Controller 的参数。`controller_manager` 参数主要包括**更新频率**以及其**加载的每个具体的 Controller 名称**，而具体的 Controller 参数则包括这个控制器控制算法所需参数。

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100      # 更新频率

    mecanum_controller:    # 需要启动的控制器
      type: mecanum_controller/MecanumController #控制器类型

    joint_state_broadcaster:  # 一个特殊的控制器，只负责向上广播关节状态
      type: joint_state_broadcaster/JointStateBroadcaster

    mecanum_controller:
      ros__parameters:       # 各类控制器构造、运算所需参数
        lf_wheel_joint_name: lf_wheel_joint
        ...
        wheel_distance:
        width: 0.208
        length: 0.22
```

### 硬件相关参数

硬件相关参数位于 URDF 文件中(需要自行添加)，主要包含两个部分。hardware 部分用于导入 `hardware interface` 和相关参数，joint 部分描述每个关节所需的接口。

```xml
<ros2_control name="mecanum" type="system">
  <hardware>
    <plugin>mecanum_hardware/MecanumHardware</plugin>
    <param name="serial_port">/dev/ttyUSB0</param>
  </hardware>
  <joint name="lf_wheel_joint">
    <command_interface name="velocity" />
    <state_interface name="position" />
    <state_interface name="velocity" />
    <param name="motor_id">1</param>
  </joint>
  <joint name="rf_wheel_joint">
    <command_interface name="velocity" />
    <state_interface name="position" />
    <state_interface name="velocity" />
    <param name="motor_id">2</param>
  </joint>
  <joint name="lr_wheel_joint">
    <command_interface name="velocity" />
    <state_interface name="position" />
    <state_interface name="velocity" />
    <param name="motor_id">3</param>
  </joint>
  <joint name="rr_wheel_joint">
    <command_interface name="velocity" />
    <state_interface name="position" />
    <state_interface name="velocity" />
    <param name="motor_id">4</param>
  </joint>
</ros2_control>
```

## 2. 编写 `Controller`

在 ros2-control 框架中，Controller 以库的形式被 `controller_manager` 通过 pluginlib 插件动态地加载。

### 创建包和文件

首先需要创建 Controller 的包，使用如下命令：

```sh
ros2 pkg create --build-type ament_cmake mecanum_controller
```

创建包以后会有 `CMakeLists.txt` 和 `package.xml` 两个文件。以及 `include/mecanum_controller` 和 `src` 两个文件夹。

- 在 `include/mecanum_controller` 中创建一个 `mecanum_controller.hpp` 文件
- 在 src 下创建 `mecanum_controller.cpp`

### 编写头文件

编写控制器头文件需要注意以下事项：

1. **设置命名空间**：一般需要使用一个 `namespace` 来保护变量，这个 `namespace` 可以用小写下划线形式(`snake case`)的控制器名，例如我们使用名为 `mecanum_controller` 的命名空间；
2. **声明类**：类名按照 ROS 规范大写为 `MecanumController`，控制器类需要继承 `ControllerInterface`；
3. **成员函数**：提供一个无参的构造函数，同时重新声明父类的 `command_interface_configuration`、`state_interface_configuration`、`on_init`、`on_configure`、`on_activate`、`on_deactivate`、`update` 等方法，后面详细介绍各个方法的作用；
4. **成员变量**：没有必须设置的成员变量，可以根据实际控制需求自定义成员变量；

### 编写源文件

源文件的核心是 `update` 方法，这个方法会**从 Hardware 读取状态，同时下发指令**。此外还需要实现一些其他函数，用于变量初始化等工作。

#### 实现 `on_init()` 方法

在 `on_init` 方法中，**一般第一行会调用基类的 init 方法**，在 init 中一般执行各类成员变量的初始化，分配内存等操作。正常情况下返回 OK，否则返回 ERROR。此函数意义不太大，在 humble 等版本中无此函数，改为 on_init 函数，一般也没有实际操作。

```cpp
/**
 * @brief 初始化函数
 * @return controller_interface::CallbackReturn
 */
controller_interface::CallbackReturn MecanumbotDriveController::on_init() {
  return controller_interface::CallbackReturn::SUCCESS;
}
```

#### 实现 `on_configure()` 方法

此方法会从 yaml 和 launch 中获取各类需要的参数，然后放到之前声明的各类成员变量中：

```cpp
/**
 * @brief 配置函数
 * @return controller_interface::CallbackReturn
 */
controller_interface::CallbackReturn MecanumController::on_configure(const rclcpp_lifecycle::State &) {
    // 获取4个轮子的实际实际名字
    lf_wheel_joint_name_ = get_node()->get_parameter("lf_wheel_joint_name").as_string();
    rf_wheel_joint_name_ = get_node()->get_parameter("rf_wheel_joint_name").as_string();
    lr_wheel_joint_name_ = get_node()->get_parameter("lr_wheel_joint_name").as_string();
    rr_wheel_joint_name_ = get_node()->get_parameter("rr_wheel_joint_name").as_string();

    ...

    // 获取麦轮结构参数
    wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
    wheel_distance_width_ = get_node()->get_parameter("wheel_distance.width").as_double();
    wheel_distance_length_ = get_node()->get_parameter("wheel_distance.length").as_double();

    ...

    // 创建各类所需的接收器和发布器
    // 详见下一节实践篇
    return controller_interface::CallbackReturn::SUCCESS;
}
```

#### 实现 状态接口(state_interface) / 命令接口(command_interface)

包括 `command_interface_configuration` 方法以及 `state_interface_configuration` 方法。此方法会返回一个 `InterfaceConfiguration` 对象，该对象描述了 command 和 state 所使用的接口类型，对象的第一个元素可以是 `ALL`，`INDIVIDUAL` 和 `NONE`，`ALL` 和 `NONE` 选项要求访问所有可用接口或者不使用任何可用接口，`INDIVIDUAL` 配置项需要一个**额外的请求接口名称列表**来确定所需接口。接口名称需要按照 `<joint_name>/<interface_type>` 的形式给出。

在本例中，需要两类状态接口和一类命令接口：

- **状态接口**：需要从硬件获取当前的关节的位置和速度；
- **命令接口**：需要向硬件设置关节当前的速度；

```cpp
/**
 * @brief command_interface_configuration
 * @return controller_interface::InterfaceConfiguration 返回一个接口配置结构体
 */
controller_interface::InterfaceConfiguration MecanumController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // 速度指令接口
  command_interfaces_config.names.push_back(lf_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  command_interfaces_config.names.push_back(rf_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  command_interfaces_config.names.push_back(lr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  command_interfaces_config.names.push_back(rr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  return command_interfaces_config;
}

/**
 * @brief state_interface_configuration
 * @return controller_interface::InterfaceConfiguration 返回一个接口配置结构体
 */
// 这里会对 `state_interfaces_` 变量进行注册吗？然后在on_activate中进行find?
// controller_interface/include/controller_interface/controller_interface_base.hpp:243:  std::vector<hardware_interface::LoanedStateInterface> state_interfaces_;
// 所以这个`state_interface_`是基类中的方法
controller_interface::InterfaceConfiguration MecanumController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // 位置状态接口、速度状态接口
  state_interfaces_config.names.push_back(lf_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  state_interfaces_config.names.push_back(lf_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  state_interfaces_config.names.push_back(rf_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  state_interfaces_config.names.push_back(rf_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  state_interfaces_config.names.push_back(lr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  state_interfaces_config.names.push_back(lr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  state_interfaces_config.names.push_back(rr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  state_interfaces_config.names.push_back(rr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  return state_interfaces_config;
}
```

#### 实现 `on_activate()` 方法

前面的 `command_interface_configuration` 方法和 `state_interface_configuration` 方法会将方法全部注册到基类对应的成员变量中，在 Controller 中一般会**为每个关节创建相应的对象**，此方法会将关节对象和其方法关联起来。

```cpp
/**
 * @brief 将关节和对应的接口关联起来
 * @return controller_interface::CallbackReturn
 */
controller_interface::CallbackReturn MecanumController::on_activate(const rclcpp_lifecycle::State &) {
    // get_wheel函数会从state_interfaces_、command_interface_中根据名称寻找对应的接口
    lf_wheel_ = get_wheel(lf_wheel_joint_name_);
    rf_wheel_ = get_wheel(rf_wheel_joint_name_);
    lr_wheel_ = get_wheel(lr_wheel_joint_name_);
    rr_wheel_ = get_wheel(rr_wheel_joint_name_);
    if (!lf_wheel_ || !rf_wheel_ || !lr_wheel_ || !rr_wheel_) {
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 根据名称，获取关节结构体
 * @param wheel_joint_name 关节名称
 * @return std::shared_ptr<MecanumWheel> 关节指针
 */
std::shared_ptr<MecanumWheel> MecanumController::get_wheel(const std::string & wheel_joint_name) {
    // position state
    const auto position_state = std::find_if(
      state_interfaces_.cbegin(),
      state_interfaces_.cend(),
      [&wheel_joint_name](const hardware_interface::LoanedStateInterface & interface) {
        return interface.get_interface_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
    });
    if (position_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position state interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // velocity state
    const auto velocity_state = std::find_if(
      state_interfaces_.cbegin(),
      state_interfaces_.cend(),
      [&wheel_joint_name](const hardware_interface::LoanedStateInterface & interface) {
        return interface.get_interface_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity state interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // velocity command
    const auto velocity_command = std::find_if(
      command_interfaces_.begin(),
      command_interfaces_.end(),
      [&wheel_joint_name](const hardware_interface::LoanedCommandInterface & interface) {
        return interface.get_interface_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_command == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity command interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    return std::make_shared<MecanumWheel>(
        std::ref(*position_state),
        std::ref(*velocity_state),
        std::ref(*velocity_command));
}
```

#### 实现 `update()` 方法

此函数运行于实时域，从硬件接口获取状态，并下发命令到硬件，一些较为简单的解算就可以直接放这里。

```cpp
/**
 * @brief 执行数据交互
 * @param time
 * @param period
 * @return controller_interface::return_type
 */
controller_interface::return_type MecanumController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period) {
    // 获取速度指令
    auto velocity_command = velocity_command_ptr_.readFromRT();
    if (!velocity_command || !(*velocity_command)) {
        return controller_interface::return_type::OK;
    }

    // 计算每个轮子的速度
    const auto twist = (*velocity_command)->twist;
    double lf_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x - twist.linear.y - (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double rf_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x + twist.linear.y + (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double lr_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x + twist.linear.y - (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double rr_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x - twist.linear.y + (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);

    // 这里是直接下发到硬件？
    // 和hardware_interface 进程进程间通信？
    lf_wheel_->set_velocity(lf_wheel_velocity);
    rf_wheel_->set_velocity(rf_wheel_velocity);
    lr_wheel_->set_velocity(lr_wheel_velocity);
    rr_wheel_->set_velocity(rr_wheel_velocity);

    return controller_interface::return_type::OK;
}
```

#### 导出 pluginlib

添加一个 `PLUGINLIB_EXPORT_CLASS` 宏，第一个参数需要提供具体的控制类 `mecanum_controller::MecanumController`，第二个参数是基类 `controller_interface::ControllerInterface`。

```cpp
PLUGINLIB_EXPORT_CLASS(
  mecanum_controller::MecanumController,
  controller_interface::ControllerInterface
)
```

### 修改 CMakeLists

```cpp
cmake_minimum_required(VERSION 3.5)
project(mecanum_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
    src/mecanum_controller.cpp
    src/mecanum_wheel.cpp
)

target_include_directories(
  ${PROJECT_NAME} PRIVATE
    include
)

ament_target_dependencies(
  ${PROJECT_NAME}
    pluginlib
    controller_interface
    hardware_interface
    realtime_tools
    rclcpp
    rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(controller_interface mecanum_controller.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  pluginlib
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)

ament_package()
```

### 修改 package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  ...

  <depend>controller_interface</depend>
  <depend>hardware_interface</depend>
  <depend>realtime_tools</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>

  <build_depend>pluginlib</build_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

完成上述操作以后，就可以执行 `colcon build` 编译这个控制器了。

## 3. 编写 `Hardware`

> [!note]
> 这里是有实际的 write() read() 方法，所以 hardware_interface 和 controller 都编译为库的形式来加载，双方之间有进程内通信？
>
> 1. 猜测是通过 `export_state_interfaces()`，`export_command_interfaces()`以及 `command_interface_configuration()`,`state_interface_configuration()`
> 2. "$ros2_control/hardware_interface/doc/writing_new_hardware_component.md"
> 3. `LoanedCommandInterface` 这里是 loan
> 4. `import_controller_reference_interfaces` 以引用的方式从接口交互信息
>
> ```cpp
>   /// Add controllers' reference interfaces to resource manager.
>   /**
>    * Interface for transferring management of reference interfaces to resource manager.
>    * When chaining controllers, reference interfaces are used as command interface of preceding
>    * controllers.
>    * Therefore, they should be managed in the same way as command interface of hardware.
>    * 用于将引用接口的管理转移到资源管理器的接口。
>    * 链接控制器时，参考接口用作前面控制器的命令接口。因此，它们应该以与硬件命令接口相同的方式进行管理。
>    * \param[in] controller_name name of the controller which reference interfaces are imported.
>    * \param[in] interfaces list of controller's reference interfaces as CommandInterfaces.
>    */
>   void import_controller_reference_interfaces(
>     const std::string & controller_name, std::vector<CommandInterface> & interfaces);
> ```
>
> 5. 同样的，hardware_interface 中 ResourceManager 类有很多 `update`-thread 的字眼，都是表明用于和 ControllerManager 进行进程内交互的
>
> ```cpp
> // CM API: Called in "update"-thread
> LoanedCommandInterface ResourceManager::claim_command_interface(const std::string & key)
> {
>   if (!command_interface_is_available(key))
>   {
>     throw std::runtime_error(std::string("Command interface with '") + key + "' does not exist");
>   }
>
>   std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
>   if (command_interface_is_claimed(key))
>   {
>     throw std::runtime_error(
>       std::string("Command interface with '") + key + "' is already claimed");
>   }
>
>   resource_storage_->claimed_command_interface_map_[key] = true;
>   std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
>   return LoanedCommandInterface(
>     resource_storage_->command_interface_map_.at(key),
>     std::bind(&ResourceManager::release_command_interface, this, key));
> }
> ```
>
> 6. controller\*interface <-> hardware_interface
>
> ```cpp
> // 在 controller_interfaces 会提供对应的方法，load 对应的 interfaces
> void ControllerInterfaceBase::assign_interfaces(
> std::vector<hardware_interface::LoanedCommandInterface> && command_interfaces,
> std::vector<hardware_interface::LoanedStateInterface> && state_interfaces)
> {
> command_interfaces* = std::forward<decltype(command*interfaces)>(command_interfaces);
> state_interfaces* = std::forward<decltype(state_interfaces)>(state_interfaces);
> }
> // controller_manager 会调用 assign，从而获得对应的 interface
> controller->assign_interfaces(std::move(command_loans), std::move(state_loans));
> ```

Hardware 同样以库的形式被 controller_manager 通过 pluginlib 插件动态地加载。本节以一个麦轮车平台讲解 Hardware Components 的编写方式。由于麦轮车具有 4 个关节，因此属于 System 类硬件。

### 创建包和文件

硬件组件需要一个单独的包，使用如下命令：

```sh
ros2 pkg create --build-type ament_cmake mecanum_hardware
```

创建包以后会有 `CMakeLists.txt` 和 `package.xml` 两个文件。以及 `include/mecanum_hardwarer` 和 `src` 两个文件夹。

- 在 `include/mecanum_hardware` 中创建一个 `mecanum_hardware.hpp` 文件
- 在 `src` 下创建 `mecanum_hardware.cpp`

### 编写头文件

编写头文件注意以下事项：

1. 导入所需的头文件。根据硬件类型(系统，执行器，传感器)的不同选择合适的头文件。例如，如果是系统类硬件，则需要写 `#include "hardware_interface/system_interface.hpp"`；
2. 编写命名空间，这个和 Controller 类似，使用小写加下划线的命名；
3. 创建类，然后根据类的类型，继承合适的数据结构，例如 `class MecanumHardware : public hardware_interface::SystemInterface`；
4. 在类中添加一系列函数声明，例如：`on_init()`，`export_state_interfaces()`，`export_command_interfaces()`，`on_activate()`，`on_deactivate()`，`read()`，`write()`。

### 编写源文件

#### 编写 `on_init()` 函数

与 controller 中类似，首先也要实现用于成员变量初始化的 `on_init()` 函数。`on_init()` 首先需要显式地调用基类的 `on_init()` 函数：`hardware_interface::(Actuator|Sensor|System)Interface::on_init(info)`，随后对各类硬件组件类需要使用的成员进行初始化。结尾运行成功返回 success。代码如下：

```cpp
/**
 * @brief 初始化函数，主要初始化一些变量
 * @param hardware_info 从urdf中读取的信息
 * @return hardware_interface::CallbackReturn
 */
hardware_interface::CallbackReturn MecanumHardware::on_init(
  const hardware_interface::HardwareInfo & hardware_info) {
  // 显式地调用基类的 `on_init()` 函数
  hardware_interface::CallbackReturn base_result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (base_result != hardware_interface::CallbackReturn::SUCCESS) {
    return base_result;
  }

  // 对各类硬件进行初始化
  serial_port_name_ = info_.hardware_parameters["serial_port"];
  motor_ids_.resize(info_.joints.size());
  position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  ...

  for (size_t i = 0; i < info_.joints.size(); i++) {
    motor_ids_[i] = (uint8_t)std::stoi(info_.joints[i].parameters["motor_id"]);
    RCLCPP_INFO(rclcpp::get_logger("MecanumbotHardware"), "%s mapped to motor %d", info_.joints[i].name.c_str(), motor_ids_[i]);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}
```

#### 实现 `export_command_interfaces()` 和 `export_state_interfaces()`

根据硬件的类型，创建所提供的接口。**这里创建的接口和 Controller 中是直接对接起来的**。每个接口都包含一个内部变量，这个变量就是实际向上层传输的数据了，代码如下：

> 通过 shared memory 实现进程内通信

```cpp
/**
 * @brief 这个函数将info_变量的接口(从URDF中进行读取)，转存到一个state_interfaces中，返回到ControllerManager
 * @return std::vector<hardware_interface::StateInterface> 返回一个状态接口列表
 */
std::vector<hardware_interface::StateInterface> MecanumHardware::export_state_interfaces() {
  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Adding position state interface: %s", info_.joints[i].name.c_str());
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }
  for (size_t i = 0; i < info_.joints.size(); i++) {
    RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Adding velocity state interface: %s", info_.joints[i].name.c_str());
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }
  return state_interfaces;
}

/**
 * @brief 同样，从info中读取命令接口，存入结构体返回到ControllerManager
 * @return std::vector<hardware_interface::CommandInterface>
 */
std::vector<hardware_interface::CommandInterface> MecanumHardware::export_command_interfaces() {
  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Adding velocity command interface: %s", info_.joints[i].name.c_str());
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }
  return command_interfaces;
}
```

请注意这里的 `velocity_states_[i]` 等数组类型变量，会和相应接口进行绑定，后续通过操作这些接口进行数据传输。

#### 编写 `on_configure()` 函数

在 `on_configure()` 函数中，一般配置到硬件的通信。这里具体情况不一，例如如果基于 Linux CAN 接口，就调用 socketCAN 的初始化程序，配置 `sockfd`；如果使用串口，那就打开文件描述符，通过 `fnctl` 进行串口通信的控制。当然，你也可以使用各类更为方便的第三方库，但是一般这里用来进行通信接口的配置相关工作。

#### 编写 `on_activate()` 函数

此函数一般使能硬件。`on_activate` 和 `on_configure` 并不是一定要实现的，例如如果你的硬件没有使能相关的协议或者接口，那就没有必要编写 `on_activate`。

#### 编写 `on_deactivate()` 函数

此函数用来**去使能硬件**，同样不是必须的。

#### 编写 `read()` 和 `write()` 函数

此两个函数用于**通过硬件连接(例如串口)读取 / 写入实际数据**，然后放在前述接口的内置变量中。

#### 导出 `pluginlib`

导出 pluginlib 和前面类似，第一个参数是`(命名空间)::(类名)`，第二个参数是其`实际继承的硬件基类`。

```cpp
PLUGINLIB_EXPORT_CLASS(
  mecanum_hardware::MecanumHardware,
  hardware_interface::SystemInterface
)
```

### 修改 CMakeLists

```sh
cmake_minimum_required(VERSION 3.8)
project(mecanum_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
    src/mecanum_hardware.cpp
)

target_include_directories(
  ${PROJECT_NAME} PRIVATE
    include
)

ament_target_dependencies(
  ${PROJECT_NAME}
    hardware_interface
    pluginlib
    rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface mecanum_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
```

### 修改 package.xml

```xml
<package format="3">
  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>hardware_interface</depend>
  <depend>rclcpp</depend>

  <build_depend>pluginlib</build_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## 4. 小结

本节介绍了如何基于 ros2-control 框架编写代码，很多内容是依据官方文档进行翻译的，官方文档比较系统全面，但偏理论性，下一节结合一个实际案例讲解具体如何实现和操作。
