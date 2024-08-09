#!/usr/bin/env python3

"""
@brief load robot in gazebo use ros2_control
@author postrantor@gmail.com
@date 2024-07-29 20:48:03

example:
   ```bash
   ros2 launch robot_description gazebo.launch.py
   ros2 run robot_description example_robot
   ```
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions.launch_configuration import LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument(
        'robot_name',
        default_value='robot_a1',
        description='robot namespace'),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='use_sim_time'),
    DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        choices=['true', 'false'],
        description='use gazebo simulation'),
    DeclareLaunchArgument(
        'debug',
        default_value='true',
        choices=['true', 'false'],
        description='debug'),
    SetEnvironmentVariable('SVGA_VGPU10', '0'),  # 禁用硬件加速
    SetEnvironmentVariable('IGNITION_FUEL_URI', ''),  # 禁止从网络下载模型文件
]


def generate_launch_description():
    # get package path
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_description = get_package_share_directory('robot_description')

    # get config file path
    gazebo_launch_file = PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
    empty_world_file = PathJoinSubstitution([pkg_description, 'config', 'worlds', 'empty.world'])
    robot_controllers = PathJoinSubstitution([pkg_description, 'config', 'controller', 'position_controller.yaml'])
    xacro_file = PathJoinSubstitution([pkg_description, 'config', 'xacro', 'robot.xacro'])
    rviz2_config_file = PathJoinSubstitution([pkg_description, 'config', 'rviz', 'robot.rviz'])

    # start Gazebo use empty.world, load gazebo_ros_state
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'world': empty_world_file}.items()
    )

    # spawn entity in gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', LaunchConfiguration('robot_name'),
            '-reference_frame', 'world',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        output='screen'
    )

    # load controller_manager with controller.yaml config
    # the loading of control_nodel conflicts with `<gazebo>` in the urdf configuration file.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_controllers': robot_controllers},
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
        output="screen",
    )

    # load robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': Command([
                'xacro', ' ', xacro_file, ' '
                'debug:=', LaunchConfiguration('debug'), ' '
                'use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'), ' '
                'gazebo:=ignition', ' ',
                'namespace:=', LaunchConfiguration('robot_name')])},
        ],
        output='screen',
    )

    # load all unitree_joint_controller
    leg_controller_names = [
        f'{leg}_{part}_controller'
        for leg in ['FR']  # 'FL', 'FR', 'RL', 'RR'
        for part in ['thigh']  # 'hip', 'thigh', 'calf'
    ]
    controller_names = ['joint_state_broadcaster'] + leg_controller_names
    load_controllers = Node(
        package="controller_manager",
        executable="spawner",
        # load each controller use controller_manager service
        arguments=['--activate-as-group', *controller_names],
        output="screen",
    )

    # load target step by step
    load_resource = TimerAction(
        period=0.0,
        actions=[gazebo, node_robot_state_publisher]
    )
    delayed_start_entity = TimerAction(
        period=10.0,
        actions=[spawn_entity]
    )
    # load controller after spawn_entity
    event_handlers = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=load_controllers))
    ]

    ld = LaunchDescription([
        *ARGUMENTS,
        load_resource,
        delayed_start_entity,
        * event_handlers,
    ])

    return ld
