"""
Demo for spawn_entity.
Launches Gazebo and spawns a model
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gazebo.launch.py']),
    )

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'demo', '-database', 'double_pendulum_with_base'],
                        output='screen')

    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])
