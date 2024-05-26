import os, xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, GroupAction, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# ARGUMENTS = [
#     DeclareLaunchArgument('use_sim_time', default_value='false',
#                           choices=['true', 'false'],
#                           description='use_sim_time'),
#     DeclareLaunchArgument('robot_name', default_value='a1_description',
#                           description='robot name'),
#     DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('robot_name'),
#                           description='robot namespace'),
# ]

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )

    pkg_description = get_package_share_directory('gazebo_ros2_control_demos')
    rviz2_config = PathJoinSubstitution([pkg_description, 'config', 'model.rviz'])
    xacro_file = PathJoinSubstitution([pkg_description, 'urdf', 'robot.urdf'])
    namespace = LaunchConfiguration('namespace')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': False},
            {'robot_description': Command(['xacro', ' ', xacro_file, ' ',])},
        ],
        output='screen',
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'diffbot'],
        output='screen')

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_base_controller'],
        output='screen'
    )

    # add rviz
    robot_rviz = GroupAction([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz2_config],
            parameters=[{'use_sim_time': False}],
            output='screen')
            ]
        )

    ld = LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_diff_drive_base_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        robot_rviz,
    ])

    return ld
