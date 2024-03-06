# ros_gz_project_template

A template project integrating ROS 2 and Gazebo simulator.

## Included packages

- `ros_gz_example_description` - holds the sdf description of the simulated system and any other assets.
- `ros_gz_example_gazebo` - holds gazebo specific code and configurations. Namely this is where systems end up.
- `ros_gz_example_application` - holds ros2 specific code and configurations.
- `ros_gz_example_bringup` - holds launch files and high level utilities.

- `ros_gz_example_description` - 保存模拟系统的 sdf 描述和任何其他资产。
- `ros_gz_example_gazebo` - 保存 gazebo 专用代码和配置。也就是说，系统在这里结束。
- `ros_gz_example_application` - 保存 ros2 的特定代码和配置。
- `ros_gz_example_bringup` - 保存启动文件和高级实用程序。

## Install

For using the template with Gazebo Fortress switch to the `fortress` branch of this repository, otherwise use the default branch `main` for Gazebo Harmonic onwards.

> 若要在 Gazebo Fortress 中使用该模板，请切换到该版本库的 `fortress` 分支，否则请在 Gazebo Harmonic 及以后版本中使用默认分支 `main`。

### Requirements

1. Choose a ROS and Gazebo combination https://gazebosim.org/docs/latest/ros_installation
   Note: If you're using a specific and unsupported Gazebo version with ROS 2, you might need to set the `GZ_VERSION` environment variable, for example:

   ```bash
   export GZ_VERSION=fortress
   ```

1. Install necessary tools

   ```bash
   sudo apt install python3-vcstool python3-colcon-common-extensions git wget
   ```

### Use as template

Directly `Use this template` and create your project repository on Github.

Or start by creating a workspace and cloning the template repository:

```bash
mkdir -p ~/template_ws/src
cd ~/template_ws/src
wget https://raw.githubusercontent.com/gazebosim/ros_gz_project_template/main/template_workspace.yaml
vcs import < template_workspace.yaml
```

## Usage

1. Install dependencies

   ```bash
   cd ~/template_ws
   source /opt/ros/<ROS_DISTRO>/setup.bash
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -r -i -y --rosdistro <ROS_DISTRO>
   ```

2. Build the project

   ```bash
   colcon build --cmake-args -DBUILD_TESTING=ON
   ```

3. Source the workspace

   ```bash
   . ~/template_ws/install/setup.sh
   ```

4. Launch the simulation

   ```bash
   ros2 launch ros_gz_example_bringup diff_drive.launch.py
   ```

5. pub

   ```sh
   ros2 topic pub --rate 30 /diff_drive/cmd_vel geometry_msgs/msg/Twist "
   linear:
      x: 0.7
      y: 0.0
      z: 0.0
   angular:
      x: 0.0
      y: 0.0
      z: 1.0"
   ```

For a more detailed guide on using this template see [documentation](https://gazebosim.org/docs/latest/ros_gz_project_template_guide).
