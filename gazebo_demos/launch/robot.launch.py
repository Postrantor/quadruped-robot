# @brief
# @author postrantor@gmail.com
# @date 2024-06-29 19:01:27
#
# example:
#    ```bash
#    ros2 launch gazebo_ros2_control_demos robot.launch.py
#    ros2 run gazebo_ros2_control_demos example_robot
#    ```

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import Command, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 获取共享目录路径
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_description = get_package_share_directory('gazebo_ros2_control_demos')

    # 文件路径定义
    gazebo_launch_file = PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
    empty_world_file = PathJoinSubstitution([pkg_description, 'worlds', 'empty.world'])
    rviz2_config_file = PathJoinSubstitution([pkg_description, 'config', 'robot.rviz'])
    xacro_file = PathJoinSubstitution([pkg_description, 'xacro', 'robot.urdf'])

    robot_controllers = PathJoinSubstitution([pkg_description, 'config', 'unitree_motor_controllers.yaml'])
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],  # same as <gazebo> tag parameters
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # 机器人状态发布器节点
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': False},
            {'robot_description': Command(['xacro', ' ', xacro_file])},
        ],
        output='screen',
    )

    # 启动 Gazebo 并指定 empty.world 文件
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'world': empty_world_file}.items()
    )

    # 生成机器人实体节点
    # 创建 `gazebo_ros_state` node，以及如下service/topic
    # - service: get_entity_state
    # - service: set_entity_state
    # - topic: model_states
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'diffdrive'],
        output='screen'
    )

    # 加载控制器
    load_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'diff_drive_base_controller', '--activate-as-group', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 启动 RViz2
    robot_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config_file],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # 事件处理器，用于在特定节点退出后启动其他节点
    event_handlers = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_controllers],)),
    ]

    # 启动描述符
    ld = LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        # robot_rviz,
        *event_handlers
    ])

    return ld
