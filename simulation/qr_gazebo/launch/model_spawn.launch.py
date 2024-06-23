# @author GPT4 bing
# @date 2024-06-23 14:55:02
# @brief 这段代码首先定义了一些参数，然后加载了关节控制器配置从YAML文件到参数服务器，接着，它将URDF加载到ROS参数服务器，最后启动了一个名为spawn_model的节点。这个Python launch文件应该能够实现与您提供的XML文件相同的功能。

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, FindPackageShare
from launch_ros.substitutions import FindPackage
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rname = LaunchConfiguration('rname', default='a1')
    robot_path = LaunchConfiguration('robot_path', default=FindPackage(rname + '_description'))
    use_xacro = LaunchConfiguration('use_xacro', default='true')
    user_debug = LaunchConfiguration('user_debug', default='false')
    use_camera = LaunchConfiguration('use_camera', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('rname', default_value=rname),
        DeclareLaunchArgument('robot_path', default_value=robot_path),
        DeclareLaunchArgument('use_xacro', default_value=use_xacro),
        DeclareLaunchArgument('user_debug', default_value=user_debug),
        DeclareLaunchArgument('use_camera', default_value=use_camera),
        ExecuteProcess(cmd=['ros2', 'param', 'load', '/robot_state_publisher', robot_path + '/config/robot_control.yaml'], output='screen'
                       ),
        ExecuteProcess(cmd=['ros2', 'param', 'set', '/robot_state_publisher', 'robot_description', Command(['cat ', robot_path + '/urdf/robot.urdf'])], condition=UnlessCondition(use_xacro), output='screen'
                       ),
        Node(package='qr_gazebo', executable='spawn_model', name='spawn_model', arguments=[rname], output='screen',
             ),
    ])
