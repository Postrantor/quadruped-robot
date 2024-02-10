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

## go-m8010-6

support motor: go-m8010-6 motor
not support motor: a1 motor、 b1 motor (check a1b1 branch for support)

```sh
cmake -Bbuild .
cmake --build build
sudo ./motorctrl
```

# Unitree MotorToolbox

This is a set of tools to operate the internal configuration of the motor through the RS-485 bus.

## Overview

`unisp`: Upgrade motor firmware .  
`changeid`: If you have multiple motors connected to a RS-485 bus, you need to change them to separate ID.  
`swboot`: Switch to Bootloader mode and then while be list all the motors.  
`swmotor`: Switch to motor mode (Default), Motor while be enable.(Configuration cannot be modified)  
`cancelboot`: Rescue modes, such as misoperation (such as upgrade failure), cause the motor to turn into brick.

### How to distinguish which mode the motor is currently in?

Look at the green LED on the back of the motor.

| Motor Mode                    | Bootloader Mode                         |
| ----------------------------- | --------------------------------------- |
| ![Motor Mode](motor_mode.gif) | ![Bootloader Mode](bootloader_mode.gif) |
| Slow Blink.                   | Fast Blink 3 times.                     |

## Using

The downloaded linux executable file does not have permission to run directly, please:  
`sudo chmod 777 ./unisp ./changeid ./swboot ./swmotor ./cancelboot`  
run example for changeid:  
`sudo ./changeid /dev/ttyUSB0 0 2`

## Other

Unitree MotorToolbox does not require root permissions to function properly, but the exception is that your `ttyUSB` device must have sufficient permissions. like:

`sudo chmod 777 /dev/ttyUSB0`
