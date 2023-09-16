---
tip: translate by baidu@2023-09-15 21:25:43
...

# rl-inference

## Introduction

This repository is an extension of the project `model-control`. It takes as input a neural network model file trained from `rl-robotics` project and runs a cpp runtime for real-world deployment. The project now supports `onnx` and `om` models for Lite3 and A1, while `rl-robotics` python project supports the `pt` model.

> 该存储库是“模型控制”项目的扩展。它将从“rl robotics”项目中训练的神经网络模型文件作为输入，并运行用于真实世界部署的 cpp 运行时。该项目现在支持 Lite3 和 A1 的“onnx”和“om”模型，而“rl robotics”python 项目支持“pt”模型。

## Software architecture

The architecture is similar to the project`model-control`, except for a new kind of locomotion state namaed as `qrFSMStateRLLocomotion`.

> 该架构类似于“模型控制”项目，除了一种新的运动状态，称为“qrFSMStateRLLomotion”。

## Prepare environment

Appropriately sized neural networks are able to be deployed on CPU, Ascend NPU or GPU onboard, determined by the computational hardware you have.

> 适当大小的神经网络可以部署在 CPU、Ascend NPU 或板载 GPU 上，这取决于您的计算硬件。

- For those having Ascend NPU, `om` model is recommended.

1. Follow the offical [guidance](https://www.hiascend.com/document/detail/zh/canncommercial/62RC1/overview/index.html) to set up the enviroment for Atlas 200I A2 device if you have a fresh SD-card.

> 1.遵循官方[指导](https://www.hiascend.com/document/detail/zh/canncommercial/62RC1/overview/index.html)如果你有一张新的 SD 卡，可以为 Atlas 200I A2 设备设置环境。

2. Since Unbuntu 22 does not support ROS1 noetic via apt tool, one needs to compile it from sources. This [tutorial](https://blog.csdn.net/Drknown/article/details/128701624) introduces ROS1 installation.

> 2.由于 Unbuntu 22 不通过 apt 工具支持 ROS1 noetic，因此需要从源代码进行编译。本[教程](https://blog.csdn.net/Drknown/article/details/128701624)介绍了 ROS1 安装。

3. Download the pre-trained models into `${PROJECT_DIR}/rl-control/rl-inference/` from the two URLs:

> 3.将预先训练好的模型从以下两个 URL 下载到“${PROJECT_DIR}/rl-control/rl-infraction/”中：

[om model for Lite3](https://ascend-repo.obs.cn-east-2.myhuaweicloud.com/Atlas%20200I%20DK%20A2/DevKit/models/23.0.RC2/lite3_320.om)

> [适用于 Lite3 的 om 型号](https://ascend-repo.obs.cn-east-2.myhuaweicloud.com/Atlas%20200I%20DK%20A2/DevKit/models/23.0.RC2/lite3_320.om)

[pt model for Lite3](https://ascend-repo.obs.cn-east-2.myhuaweicloud.com/Atlas%20200I%20DK%20A2/DevKit/models/23.0.RC2/lite3_320.pt)

> [pt 型号适用于 Lite3](https://ascend-repo.obs.cn-east-2.myhuaweicloud.com/Atlas%20200I%20DK%20A2/DevKit/models/23.0.RC2/lite3_320.pt)

- For those only having CPU(x86 or arm) device, the `onnx` model is runnable. You can follow this [website](https://stackoverflow.com/questions/63420533/setting-up-onnx-runtime-on-ubuntu-20-04-c-api) to install ONNX-Runtime firstly.

> -对于那些只有 CPU（x86 或 arm）设备的设备，“onnx”模型是可运行的。您可以关注此[网站](https://stackoverflow.com/questions/63420533/setting-up-onnx-runtime-on-ubuntu-20-04-c-api)首先安装 ONNX Runtime。

Note that the `onnx` model is converted from the `pt` model and the `om` model is further converted from the `onnx` model using [ATC](https://www.hiascend.com/document/detail/zh/canncommercial/62RC1/inferapplicationdev/atctool/atctool_000014.html) command.

> 注意，“onnx”模型是从“pt”模型转换而来的，“om”模型是使用[ATC]从“onnx”模型进一步转换而来(https://www.hiascend.com/document/detail/zh/canncommercial/62RC1/inferapplicationdev/atctool/atctool_000014.html)命令。

# Usage

### Run inference on Ascend Atlas 200I (NPU) in the real-world.

First of all, go into the directory `rl-inference`.

Take Lite3 for example. To compile the source for om inference, one should turn on the cmake variable `USE_NPU` in the file `rl-inference/src/quadruped/CMakeLists.txt`, as well as trun off the variable `USE_ONNX`. Set the value of `network_path` in the configuration file `src/quadruped/config/lite3/main.yaml` to the right model path. Then compile the code:

> 以 Lite3 为例。要编译 om 推理的源代码，应该打开文件“rl-intrition/src/tquadruped/CMakeLists.txt”中的 cmake 变量“USE_NPU”，并关闭变量“USE_ONNX”。将配置文件“src/quadruped/config/lite3/main.yaml”中的“network_path”值设置为正确的模型路径。然后编译代码：

```
catkin_make --only-pkg-with-deps quadruped examples
```

As with `model-control`,

```
source devel/setup.bash
rosrun examples example_lite3_real
```

After launching the control process, you need to wait until the quadruped stands up, followed by pressing B key on Logitech F710 gamepad to switch to toque-stance mode. Then press the X key to switch to RL-locomotion mode with network inference.

> 启动控制过程后，您需要等待四足动物站起来，然后按 Logitech F710 游戏板上的 B 键切换到扭腿站立模式。然后按 X 键切换到具有网络推理的 RL 运动模式。

### Joy Control (Only for Lite3)

If you do not have Logitech F710 gamepad, a lite3 gamepad can be used to control the quadruped at runtime. The detailed command logic is coded in the function `qrDesiredStateCommand::RecvSocket`.

> 如果您没有 Logitech F710 游戏板，则可以使用 lite3 游戏板在运行时控制四足动物。详细的命令逻辑在函数`qrDesiredStateCommand:：RecvSocket`中进行编码。
