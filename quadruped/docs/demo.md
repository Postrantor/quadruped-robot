---
title: Demos
---

Once you have conquered the HelloWorld example, either in simulation or in real environment, you should start looking at other demo examples. Each demo\'s name is formed of multiple words that are joined together to clearly hint what it does. These words are separating using "_" (underscore). Each demo\'s name begins with a word [demo_]{.title-ref}, followed by two or three keywords. We encourage you to explore and make changes to these demos.

> 一旦你在模拟或真实环境中掌握了 HelloWorld 示例，就应该开始查看其他演示示例。每个 demo 的名称都由多个单词组成，这些单词连在一起，清楚地暗示了它的作用。这些单词之间用 "_"(下划线)分隔。每个 demo\ 的名称都以 demo_ 开头，后面跟着两三个关键词。我们鼓励您对这些演示进行探索和修改。

# demo_trot_velocity

This demo shows the trotting gait of a quadruped robot on a flat ground. Through controlling the forward linear speed and rotating speed of the body or a quadruped robot, the body\'s trajectory and the supporting foot\'s position are computed using `LocomotionMode::VELOCITY_LOCOMOTION` for the next gait cycle.

> 该演示展示了四足机器人在平地上的小跑步态。通过控制身体或四足机器人的前进线速度和旋转速度，使用 `LocomotionMode::VELOCITY_LOCOMOTION` 计算出下一个步态周期的身体轨迹和支撑脚的位置。

### demo_trot_velocity_mpc

This demo shows the trotting gait of a quadruped robot on a flat ground. Through controlling the forward linear speed and rotating speed of the body or a quadruped robot, the body\'s trajectory and the supporting foot\'s position are computed using `LocomotionMode::VELOCITY_LOCOMOTION` for the next gait cycle. Here, model predictive control (MPC) technique is used.

> 该演示展示了四足机器人在平地上的小跑步态。通过控制身体或四足机器人的前进线速度和旋转速度，使用 `LocomotionMode::VELOCITY_LOCOMOTION` 计算出下一个步态周期的身体轨迹和支撑脚的位置。这里使用的是模型预测控制(MPC)技术。

# demo_trot_position

This demo shows the trotting gait of a quadruped robot on a flat ground. Through solving inverse kinematics, the body keeps balanced and the joint angles (by inverse kinematics) or torques (by inverse dyanmics) are computed using `LocomotionMode::POSITION_LOCOMOTION`.

> 该演示展示了四足机器人在平地上的小跑步态。通过求解逆运动学，身体保持平衡，并使用 `LocomotionMode::POSITION_LOCOMOTION` 计算关节角度(通过逆运动学)或扭矩(通过逆运动学)。

# demo_trot_keyboard

This demo shows the trotting gait of a quadruped robot on a flat ground. You can use a keyboard to control the forward speed and rotating speed.

> 该演示展示了四足机器人在平地上的小跑步态。您可以使用键盘控制前进速度和旋转速度。

# demo_trot_joystick

This demo shows the trotting gait of a quadruped robot on a flat ground. You can use a game joystick to control the forward speed and rotating speed.

> 该演示展示了四足机器人在平地上小跑的步态。您可以使用游戏操纵杆控制前进速度和旋转速度。

### demo_external_force

This demo shows how to apply external forces to a moving robot. Start this program when another demo has already started.

> 本演示介绍如何对移动机器人施加外力。当另一个演示已经开始时，请启动此程序。

### demo_slam_publish_odom

This demo shows how to pulish odemetry data.

# demo_slam_gmapping

This demo shows how to build a map using SLAM and gmapping.

### demo_slam_cartographer

This demo shows how to build a map using SLAM and cartographer.

# demo_terrain_slope

This demo shows how to travel onto a slope.

# demo_terrain_stair

This demo shows how to travel onto stairs.

# demo_terrain_rough

This demo shows how to travel in a rough ground.

### demo_terrain_groove

This demo shows how to walk inside a groove.
