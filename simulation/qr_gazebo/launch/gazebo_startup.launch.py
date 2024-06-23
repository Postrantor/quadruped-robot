# @author GPT4 bing
# @date 2024-06-23 14:55:02

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from gazebo_ros.actions import GazeboRosPaths, Gazebo


def generate_launch_description():
    wname = LaunchConfiguration('wname', default='earth')
    paused = LaunchConfiguration('paused', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    headless = LaunchConfiguration('headless', default='false')
    debug = LaunchConfiguration('debug', default='false')
    verbose = LaunchConfiguration('verbose', default='false')

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', FindPackageShare('qr_gazebo') + '/worlds/building_editor_models'),
        DeclareLaunchArgument('wname', default_value=wname),
        DeclareLaunchArgument('paused', default_value=paused),
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        DeclareLaunchArgument('gui', default_value=gui),
        DeclareLaunchArgument('headless', default_value=headless),
        DeclareLaunchArgument('debug', default_value=debug),
        DeclareLaunchArgument('verbose', default_value=verbose),
        GazeboRosPaths(
            package='qr_gazebo',
            gazebo_model_path=[FindPackageShare('qr_gazebo') + '/worlds/building_editor_models'],
            gazebo_media_path=[FindPackageShare('qr_gazebo') + '/worlds/building_editor_models'],
            gazebo_resource_path=[FindPackageShare('qr_gazebo') + '/worlds/building_editor_models'],
        ),
        Gazebo(
            world=FindPackageShare('qr_gazebo') + '/worlds/' + wname + '.world',
            debug=debug,
            gui=gui,
            paused=paused,
            use_sim_time=use_sim_time,
            headless=headless,
        ),
        Node(
            package='controller_manager',
            executable='controller_manager',
            name='controllers_manager',
            output='screen',
        ),
    ])
