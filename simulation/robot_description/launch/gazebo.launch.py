"""
@brief
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
        default_value='false',
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
    empty_world_file = PathJoinSubstitution([pkg_description, 'worlds', 'earth.world'])
    rviz2_config_file = PathJoinSubstitution([pkg_description, 'config', 'robot.rviz'])
    robot_controllers = PathJoinSubstitution([pkg_description, 'config', 'controller.yaml'])
    xacro_file = PathJoinSubstitution([pkg_description, 'xacro', 'robot.xacro'])

    # start Gazebo use empty.world, load gazebo_ros_state
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'world': empty_world_file}.items()
    )

    # load robot_state_publisher
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
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen',
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],  # same as <gazebo> tag parameters
        output="screen",
    )

    # spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', LaunchConfiguration('robot_name')],
        output='screen'
    )

    # load all controller
    leg_controller_names = [
        f'{leg}_{part}_controller'
        for leg in ['FL', 'FR', 'RL', 'RR']
        for part in ['hip', 'thigh', 'calf']
    ]
    controller_names = ['FL_thigh_controller'] + ['joint_state_broadcaster']
    load_controllers = Node(
        package="controller_manager",
        executable="spawner",
        # load each controller -> config and active all controller
        arguments=['--activate-as-group', *controller_names, '--controller-manager', '/controller_manager'],
        output="screen",
    )

    # load target
    load_resource = TimerAction(
        period=0.0,
        actions=[gazebo]
    )
    delayed_start_entity = TimerAction(
        period=5.0,
        actions=[node_robot_state_publisher, spawn_entity]
    )
    event_handlers = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,  # load_and_config_controllers[-1],
                on_exit=load_controllers
            ))
    ]

    ld = LaunchDescription([
        *ARGUMENTS,
        load_resource,
        delayed_start_entity,
        *event_handlers,
    ])

    return ld
