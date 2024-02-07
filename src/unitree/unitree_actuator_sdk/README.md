---
tip: translate by baidu@2024-01-24 23:01:37
---

# Introduction

This library is used for the communication between PC and the motor control board, so that the user can control the motors by PC. This library includes Linux and Windows version, where the Linux version also support ROS. And we offer the usage examples of C, C++ and Python.

> 此库用于 PC 与电机控制板之间的通信，以便用户可以通过 PC 控制电机。此库包括 Linux 和 Windows 版本，其中 Linux 版本也支持 ROS。我们还提供了 C、C++ 和 Python 的使用示例。

# Dependencies

- [CMake](http://www.cmake.org) (version 2.8.3 or higher)

# Files

## /lib

Including the library files of Linux and Windows separately. If you wish to use the SDK under ARM32/64, please modify the CMakeList manually to select the correst `.so` file.

> 分别包括 Linux 和 Windows 的库文件。如果您希望使用 ARM32/64 下的 SDK，请手动修改 CMakeList 以选择相应的`.so`文件。

## /include

Including the head files. Where `LSerial.h` contains the declarations of serial port operation functions. The `motor_msg.h` contains the command structure for motor communication. The `motor_ctrl.h` contains the encoding and decoding functions.

> 包括头文件。其中`LSerial.h`包含串行端口操作函数的声明。`motor_msg.h`包含用于电机通信的命令结构。`motor_ctrl.h`包含编码和解码功能。

## /ChangeID_Tool

Including the tool to change motor's ID of Linux and Windows separately. Please follow guidances of the executable file.

> 包括分别更改 Linux 和 Windows 的电机 ID 的工具。请遵循可执行文件的指导。

## /src

Including the example source files of C and C++. The example can control motors to run under desired command for desired time, and then stop. Please watch out that only the check.c is the full control example with all functions. check.cpp and also the Python example do not contain some comments and important tutorials.

> 包括 C 和 C++的示例源文件。该示例可以控制电机在所需的命令下运行所需的时间，然后停止。请注意，只有 check.c 是具有所有功能的完整控制示例。check.cpp 和 Python 示例都不包含一些注释和重要教程。

## /script

Including the example source files of Python. This example's function is same with the C/C++ example. The `typedef.py` declares the data structure of all library functions and structures in Python style, so that the `check.py` can call the library correctly.

> 包括 Python 的示例源文件。此示例的功能与 C/C++示例相同。`typedef.py`以 Python 样式声明所有库函数和结构的数据结构，这样`check.py`就可以正确地调用库。

## /unitree_motor_ctrl

It is a package of ROS and the library file is just the library file of Linux.

> 它是 ROS 的一个包，库文件只是 Linux 的库文件。

## /build

Build directory

## /bin

The directory of final executable file.

# Usage

## C/C++ under Liunx

### Build

```bash
mkdir build
cd build
cmake ..
make
```

### Run

```bash
cd ../bin
sudo ./check_c
```

and

```bash
cd ../bin
sudo ./check_cpp
```

## Python under Linux

```bash
cd script
sudo python3 check.py
```

## C/C++ under ROS

### Build

Under the catkin workspace, run:

```bash
catkin_make
```

### Run

As we cannot use `sudo rosrun`, please run as follows:

First, at a terminal, run:

```bash
roscore
```

In another terminal, run:

```bash
sudo su
source devel/setup.bash
rosrun unitree_motor_ctrl unitree_motor_ctrl_node
```

## Python under ROS

First, at a terminal, run:

```bash
roscore
```

In another terminal, run:

```bash
sudo su
source devel/setup.bash
rosrun unitree_motor_ctrl check.py
```

## C/C++ under Windows

### Build

We take MinGW as an example. First, select the "MinGW Makefiles" in CMake GUI and generate the makefiles to `build` directory. Then open the cmd.exe, run:

> 我们以 MinGW 为例。首先，在 CMake GUI 中选择`MinGW Makefiles`，并将 Makefiles 生成到`build`目录。然后打开 cmd.exe，运行：

```bash
cd build
mingw32-make.exe
```

### Run

Put the generated .exe file(/bin) and the .dll file(/lib) to same directory, and double click the .exe file to run.

> 将生成的.exe 文件（/bin）和.dll 文件（/lib）放在同一目录下，双击该.exe 文件即可运行。

## Python under Windows

Open the cmd.exe, then:

```bash
cd script
check.py
```
