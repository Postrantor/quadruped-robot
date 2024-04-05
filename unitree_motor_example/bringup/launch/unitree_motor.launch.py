#!/usr/bin/env python
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # %% declare arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument(
        "gui", default_value="true",
        description="start rviz2 automatically with this launch file.",))
    declared_arguments.append(DeclareLaunchArgument(
        "use_mock_hardware", default_value="false",
        description="start robot with mock hardware mirroring command to its states.", ))
    # initialize arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # get urdf from description/urdf/*.xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("unitree_motor_example"), "urdf", "unitree_motor.urdf.xacro"]), " ",])
    robot_description = {"robot_description": robot_description_content}
    # get controller param from bringup/config/*.yaml
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("unitree_motor_example"), "config", "unitree_motor_controllers.yaml",])
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("model_description"), "unitree/rviz", "unitree.rviz"])

    # %% launch node
    # FIXME(@zhiqi.jia), 不太理解为什么在这里进行映射
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[("/unitree_position_controller/desired_state", "/cmd_vel"),],)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),)
    # 通过 controller_manager启动controller，同时指定命名空间 `/controller_manager`?
    # 在这里将yaml中的参数传给controller_manager，之后就可以管理其中的 controller
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",)
    # 从bringup/config/*.yaml 中读取的参数？
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],)
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["unitree_base_controller", "--controller-manager", "/controller_manager"],)

    # %% 确定启动顺序
    # delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],))
    # delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],))

    nodes = [
        control_node,
        robot_controller_spawner,
        #   robot_state_pub_node,
        #   joint_state_broadcaster_spawner,
        #   delay_rviz_after_joint_state_broadcaster_spawner,
        #   delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
