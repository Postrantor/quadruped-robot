---
tip: translate by baidu@2024-02-12 19:31:57
github_url: <https://github.com/ros-controls/ros2_controllers/blob/%7BREPOS_FILE_BRANCH%7D/doc/writing_new_controller.rst>
---

# Writing a new controller {#writing_new_controllers}

In this framework controllers are libraries, dynamically loaded by the controller manager using the [pluginlib](https://docs.ros.org/en/%7BDISTRO%7D/Tutorials/Beginner-Client-Libraries/Pluginlib.html) interface. The following is a step-by-step guide to create source files, basic tests, and compile rules for a new controller.

> 在这个框架中，控制器是库，由控制器管理器使用[pluginlib]动态加载接口。以下是为新控制器创建源文件、基本测试和编译规则的分步指南。

## 1. **Preparing package**

If the package for the controller does not exist, then create it first. The package should have `ament_cmake` as a build type. The easiest way is to search online for the most recent manual. A helpful command to support this process is `ros2 pkg create`. Use the `--help` flag for more information on how to use it. There is also an option to create library source files and compile rules to help you in the following steps.

> 如果控制器的包不存在，请先创建它。包应具有`ament_cmake`作为生成类型。最简单的方法是在线搜索最新的手册。支持此过程的一个有用命令是`ros2-pkg create`。有关如何使用它的详细信息，请使用`--help`标志。还有一个选项可以创建库源文件和编译规则，以帮助您完成以下步骤。

## 2. **Preparing source files**

After creating the package, you should have at least `CMakeLists.txt` and `package.xml` files in it. Create also `include/<PACKAGE_NAME>/` and `src` folders if they do not already exist. In `include/<PACKAGE_NAME>/` folder add `<controller_name>.hpp` and `<controller_name>.cpp` in the `src` folder. Optionally add `visibility_control.h` with the definition of export rules for Windows. You can copy this file from an existing controller package and change the name prefix to the `<PACKAGE_NAME>`.

> 创建包后，其中至少应包含`CMakeLists.txt`和`package.xml`文件。如果还不存在`include/<package_NAME>/`和`src`文件夹，请创建它们。在`include/<PACKAGE_NAME>/`文件夹中，将`<controller_NAME>.hpp`和`<controler_NAME>.cpp`添加到`src`文件夹中。(可选)添加`visibility_control.h`和 Windows 导出规则的定义。您可以从现有的控制器包中复制此文件，并将名称前缀更改为`<package_name>`。

## 3. **Adding declarations into header file (.hpp)**

1. Take care that you use header guards. ROS 2-style is using `#ifndef` and `#define` preprocessor directives. (For more information on this, a search engine is your friend :) ).
2. include `"controller_interface/controller_interface.hpp"` and `visibility_control.h` if you are using one.
3. Define a unique namespace for your controller. This is usually a package name written in `snake_case`.
4. Define the class of the controller, extending `ControllerInterface`, e.g.,
   ```cpp
   class ControllerName : public controller_interface::ControllerInterface
   ```
5. Add a constructor without parameters and the following public methods overriding the `ControllerInterface` definition: `on_init`, `command_interface_configuration`, `state_interface_configuration`, `on_configure`, `on_activate`, `on_deactivate`, `update`. For exact definitions check the `controller_interface/controller_interface.hpp` header or one of the controllers from [ros2_controllers](https://github.com/ros-controls/ros2_controllers).
6. (Optional) The NodeOptions of the LifecycleNode can be personalized by overriding the default method `get_node_options`.
7. (Optional) Often, controllers accept lists of joint names and interface names as parameters. If so, you can add two protected string vectors to store those values.

> 1. 小心使用收割台护罩。ROS 2 样式使用`#ifndef`和`#define`预处理器指令。(关于这方面的更多信息，搜索引擎是你的朋友：)。
> 2. 包括`controller_interface/controller_interface.hpp`和`visibility_control.h`(如果您正在使用)。
> 3. 为控制器定义一个唯一的命名空间。这通常是用`snake_case`编写的包名称。
> 4. 定义控制器的类，扩展`ControllerInterface`，例如：
> 5. 添加一个不带参数的构造函数和覆盖`ControllerInterface`定义的以下公共方法：`on_init`、`command_interface_configuration`、`state_interface-configuration`、`on_configure`、`on_nactivate`、`on_deactivate`和`update`。有关确切定义，请检查`controller_interface/controller_interface.hpp `标头或[ros2_controllers]中的一个控制器.
> 6. (可选)LifecycleNode 的 NodeOptions 可以通过覆盖默认方法`get_node_options`进行个性化设置。
> 7. (可选)控制器通常接受关节名称和接口名称的列表作为参数。如果是这样，可以添加两个受保护的字符串向量来存储这些值。

### Adding definitions into source file (.cpp)

1. Include the header file of your controller and add a namespace definition to simplify further development.
2. (optional) Implement a constructor if needed. There, you could initialize member variables. This could also be done in the `on_init` method.
3. Implement the `on_init` method. The first line usually calls the parent `on_init` method. Here is the best place to initialize the variables, reserve memory, and most importantly, declare node parameters used by the controller. If everything works fine return `controller_interface::return_type::OK` or `controller_interface::return_type::ERROR` otherwise.
4. Write the `on_configure` method. Parameters are usually read here, and everything is prepared so that the controller can be started.
5. Implement `command_interface_configuration` and `state_interface_configuration` where required interfaces are defined. There are three options of the interface configuration `ALL`, `INDIVIDUAL`, and `NONE` defined in `controller_interface/controller_interface.hpp"`. `ALL` and `NONE` option will ask for access to all available interfaces or none of them. The `INDIVIDUAL` configuration needs a detailed list of required interface names. Those are usually provided as parameters. The full interface names have structure `<joint_name>/<interface_type>`.
6. Implement the `on_activate` method with checking, and potentially sorting, the interfaces and assigning members\' initial values. This method is part of the real-time loop, therefore avoid any reservation of memory and, in general, keep it as short as possible.
7. Implement the `on_deactivate` method, which does the opposite of `on_activate`. In many cases, this method is empty. This method should also be real-time safe as much as possible.
8. Implement the `update` method as the main entry point. The method should be implemented with [real-time](https://en.wikipedia.org/wiki/Real-time_computing) constraints in mind. When this method is called, the state interfaces have the most recent values from the hardware, and new commands for the hardware should be written into command interfaces.
9. IMPORTANT: At the end of your file after the namespace is closed, add the `PLUGINLIB_EXPORT_CLASS` macro. For this you will need to include the `"pluginlib/class_list_macros.hpp"` header. As first parameters you should provide exact controller class, e.g., `<controller_name_namespace>::<ControllerName>`, and as second the base class, i.e., `controller_interface::ControllerInterface`.

> 1. 包括控制器的头文件，并添加命名空间定义以简化进一步的开发。
> 2. (可选)根据需要实现构造函数。在那里，您可以初始化成员变量。这也可以在`on_init`方法中完成。
> 3. 实现`on_init`方法。第一行通常调用父`on_init`方法。这里是初始化变量、保留内存以及最重要的是声明控制器使用的节点参数的最佳位置。如果一切正常，则返回`controller_interface::return_type::OK`，否则返回`controller-interface::returntype::ERROR`。
> 4. 编写`on_configure`方法。这里通常会读取参数，并准备好一切，以便启动控制器。
> 5. 在定义了所需接口的情况下，实现`command_interface_configuration`和`state_interface-configuration`。接口配置有三个选项`ALL`、`INDIVIDUAL`和`NONE`，这些选项定义在`controller_interface/controller_interface.hpp`中。`ALL`和`NONE`选项将要求访问所有可用接口或不访问任何可用接口。`INDIVIDUAL`配置需要所需接口名称的详细列表。这些名称通常作为参数提供。完整接口名称的结构为`<joint_name>/<interface_type>`。
> 6. 实现`on_activate`方法，检查并可能对接口进行排序，并分配成员的初始值。这种方法是实时循环的一部分，因此避免了任何内存预留，并且通常尽可能短。
> 7. 实现`on_dectivate`方法，其作用与`on_deactivate`相反。在许多情况下，此方法是空的。这种方法也应该尽可能地实时安全。
> 8. 以`update`方法作为主要切入点。该方法应使用[实时]实现。当调用此方法时，状态接口具有来自硬件的最新值，并且应该将硬件的新命令写入命令接口。
> 9. 重要：在命名空间关闭后的文件末尾，添加`PLUGINILIB_EXPORT_CLASS`宏。需要包含`pluginlib/class_list_macros.hpp`。作为第一个参数，应该提供确切的控制器类，例如`<controller_name_namespace>::<ControllerName>`，作为第二个参数，应提供基类，即`controller_interface::ControllerInterface`。

### Writing export definition for pluginlib

1. Create the `<controller_name>.xml` file in the package and add a definition of the library and controller\'s class which has to be visible for the pluginlib. The easiest way to do that is to check other controllers in the [ros2_controllers](https://github.com/ros-controls/ros2_controllers) package.
2. Usually, the plugin name is defined by the package (namespace) and the class name, e.g., `<controller_name_package>/<ControllerName>`. This name defines the controller\'s type when the controller manager searches for it. The other two parameters have to correspond to the definition done in macro at the bottom of the `<controller_name>.cpp` file.

> 1. 在包中创建`<controller_name>.xml`文件，并添加库和控制器类的定义，该定义必须对 pluginlib 可见。最简单的方法是检查[ros2_controllers]中的其他控制器。
> 2. 通常，插件名称由包(名称空间)和类名定义，例如`<controller_name_package>/<ControllerName>`。当控制器管理器搜索控制器时，此名称定义控制器的类型。其他两个参数必须与`<controller_name>.cpp`文件底部宏中的定义相对应。

### Writing simple test to check if the controller can be found and loaded

1. Create the folder `test` in your package, if it does not exist already, and add a file named `test_load_<controller_name>.cpp`.
2. You can safely copy the file\'s content for any controller defined in the [ros2_controllers](https://github.com/ros-controls/ros2_controllers) package.
3. Change the name of the copied test and in the last line, where controller type is specified put the name defined in `<controller_name>.xml` file, e.g., `<controller_name_package>/<ControllerName>`.

> 1. 在包中创建文件夹`test`(如果它还不存在)，并添加一个名为`test_load_<controller_name>.cpp`的文件。
> 2. 您可以安全地复制[ros2_controllers]中定义的任何控制器的文件内容
> 3. 更改复制的测试的名称，并在最后一行中，在指定控制器类型的地方，放入`<controller_name>.xml'文件`中定义的名称，例如`<controler_name_package>/<ControllerName>'。

### Add compile directives into `CMakeLists.txt` file

1. Under the line `find_package(ament_cmake REQUIRED)` add further dependencies. Those are at least: `controller_interface`, `hardware_interface`, `pluginlib`, `rclcpp` and `rclcpp_lifecycle`.
2. Add a compile directive for a shared library providing the `<controller_name>.cpp` file as the source.
3. Add targeted include directories for the library. This is usually only `include`.
4. Add ament dependencies needed by the library. You should add at least those listed under 1.
5. Export for pluginlib description file using the following command:
   ```cmake
   pluginlib_export_plugin_description_file(controller_interface <controller_name>.xml)
   ```
6. Add install directives for targets and include directory.
7. In the test section add the following dependencies: `ament_cmake_gmock`, `controller_manager`, `hardware_interface`, `ros2_control_test_assets`.
8. Add compile definitions for the tests using the `ament_add_gmock` directive. For details, see how it is done for controllers in the [ros2_controllers](https://github.com/ros-controls/ros2_controllers) package.
9. (optional) Add your controller\`s library into `ament_export_libraries` before `ament_package()`.

> 1. 在`find_package(ament_cmake REQUIRED)`行下添加更多的依赖项。它们至少是：`controller_interface`、`hardware_interface`、`pluginlib`、`rclcpp`和`rclpp_lifecycle`。
> 2. 为提供`<controller_name>.cpp`文件作为源的共享库添加编译指令。
> 3. 为库添加目标包含目录。这通常只是`include`。
> 4. 添加库所需的辅助项。您应该至少添加 1 下列出的那些。
> 5. 使用以下命令导出 pluginlib 说明文件：
> 6. 为目标和 include 目录添加安装指令。
> 7. 在测试部分中添加以下依赖项：`ament_cmake_gmock`、`controller_manager`、`hardware_interface`、`ros2_control_test_assets`。
> 8. 使用`ament_add_gmock`指令为测试添加编译定义。有关详细信息，请参阅如何在[ros2_controllers]中为控制器执行此操作
> 9. (可选)将控制器的库添加到`ament_package()`之前的`ament_export_libraries`中。

### Add dependencies into `package.xml` file

1. Add at least the following packages into `<depend>` tag: `controller_interface`, `hardware_interface`, `pluginlib`, `rclcpp` and `rclcpp_lifecycle`.

> 1. 在`<depend>`标记中至少添加以下包：`controller_interface`、`hardware_interface`、`pluginlib`、`rclcpp`和`rclcpcpp_lifecycle`。

2. Add at least the following package into `<test_depend>` tag: `ament_add_gmock`, `controller_manager`, `hardware_interface`, and `ros2_control_test_assets`.

> 2.在`<test_depend>`tag 中至少添加以下包：`ament_add_gmock`、`controller_manager`、`hardware_interface`和`ros2_control_test_assets`。

### Compiling and testing the controller

1. Now everything is ready to compile the controller using the `colcon build <controller_name_package>` command. Remember to go into the root of your workspace before executing this command.
2. If compilation was successful, source the `setup.bash` file from the install folder and execute `colcon test <controller_name_package>` to check if the new controller can be found through `pluginlib` library and be loaded by the controller manager.

> 1. 现在一切就绪，可以使用`colcon build<controller_name_package>`命令编译控制器。在执行此命令之前，请记住进入工作区的根目录。
> 2. 如果编译成功，请从安装文件夹中获取`setup.bash`文件，并执行`colcon test ＜controller_name_package＞`以检查是否可以通过`pluginlib`库找到新控制器并由控制器管理器加载。

That\'s it! Enjoy writing great controllers!

## Useful External References

- [Templates and scripts for generating controllers shell](https://rtw.stoglrobotics.de/master/use-cases/ros2_control/setup_controller.html)
