## gazebo_ros_pkgs

[![Build Status](http://build.ros.org/buildStatus/icon?job=Kpr__gazebo_ros_pkgs__ubuntu_xenial_amd64)](http://build.ros.org/job/Kpr__gazebo_ros_pkgs__ubuntu_xenial_amd64)

Wrappers, tools and additional API's for using ROS with the Gazebo simulator. Formally simulator_gazebo stack, gazebo_pkgs is a meta package. Now Catkinized and works with the standalone Gazebo debian.

### Installation

[Installing gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)

### Documentation and Tutorials

[On gazebosim.org](http://gazebosim.org/tutorials?cat=connect_ros)

### Develop and Contribute

See [Contribute](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/CONTRIBUTING.md) page.

### build

The ros_gz library makes heavy use of templates which causes compilers to consume a lot of memory. If your build fails with c++: fatal error: Killed signal terminated program cc1plus try building with colcon build --parallel-workers=1 --executor sequential. You might also have to set export MAKEFLAGS="-j 1" before running colcon build to limit the number of processors used to build a single package.

> ros_gz 库大量使用模板，导致编译器消耗大量内存。如果您的构建因 c++: fatal error: Killed signal terminated program cc1plus 失败，请尝试使用 colcon build --parallel-workers=1 --executor sequential 构建。您可能还必须在运行 colcon build 之前设置 export MAKEFLAGS="-j 1" 以限制用于构建单个包的处理器数量。

> https://github.com/gazebosim/ros_gz/blob/ros2/README.md
