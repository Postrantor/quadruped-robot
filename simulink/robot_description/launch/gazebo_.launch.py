import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_a1_sim = get_package_share_directory('a1_sim')

    gazebo_params_file = os.path.join(pkg_a1_sim, 'config', 'gazebo_params.yaml')
    robot_description_file = os.path.join(pkg_a1_sim, 'urdf', 'a1_sim.urdf.xacro')
    controller_config_file = os.path.join(pkg_a1_sim, 'config', 'a1_controller.yaml')

    # Define the controllers list to be loaded
    controller_names = [
        'joint_state_broadcaster',
        'FL_hip_controller',
        'FL_thigh_controller',
        'FL_calf_controller',
        'FR_hip_controller',
        'FR_thigh_controller',
        'FR_calf_controller',
        'RL_hip_controller',
        'RL_thigh_controller',
        'RL_calf_controller',
        'RR_hip_controller',
        'RR_thigh_controller',
        'RR_calf_controller'
    ]

    # Create spawner nodes for each controller
    spawner_nodes = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller_name, '--controller-manager', '/controller_manager'],
            output='screen'
        ) for controller_name in controller_names
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
            launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'robot_description': Command(['xacro ', robot_description_file])}]),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_config_file],
            output='screen'),

        # Add all spawner nodes to the launch description
        *spawner_nodes
    ])
