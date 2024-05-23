"""
Launch Gazebo server and client with command line arguments.
"""

from launch import LaunchDescription  # 用于描述launch文件中的各个元素
from launch.actions import DeclareLaunchArgument  # 用于声明可以从命令行传递的参数
from launch.actions import IncludeLaunchDescription  # 用于包含其他的launch文件
from launch.conditions import IfCondition  # 用于条件性地执行某些launch文件或动作
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 用于加载Python格式的launch文件
from launch.substitutions import LaunchConfiguration  # 用于获取launch参数的值
from launch.substitutions import ThisLaunchFileDir  # 用于获取当前launch文件所在的目录


# 定义生成launch描述的方法
def generate_launch_description():

    return LaunchDescription([
        # 声明launch参数
        # `gui` 参数：默认值为 `true`，用于控制是否启动Gazebo客户端（GUI）。如果设置为 `false`，则不启动GUI模式。
        DeclareLaunchArgument('gui', default_value='true',
                              description='Set to "false" to run headless.'),
        # `server` 参数：默认值为 `true`，用于控制是否启动Gazebo服务器。如果设置为 `false`，则不启动服务器。
        DeclareLaunchArgument('server', default_value='true',
                              description='Set to "false" not to run gzserver.'),

        # 4. 包含其他launch文件
        # `gzserver.launch.py`：如果 `server` 参数为 `true`，则包含并执行该launch文件，启动Gazebo服务器。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzserver.launch.py']),
            condition=IfCondition(LaunchConfiguration('server'))),
        # `gzclient.launch.py`：如果 `gui` 参数为 `true`，则包含并执行该launch文件，启动Gazebo客户端（GUI）。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzclient.launch.py']),
            condition=IfCondition(LaunchConfiguration('gui'))),
    ])
