## how to run

```bash
ros2 launch gazebo_ros2_control_demos robot.launch.py
ros2 run gazebo_ros2_control_demos example_robot
```

## topic/service

在通过 launch 启动 gazebo 之后，可以在终端中查看到如下的 Topic/Service

### topic

```bash
trantor ➜ ~/project/demo_ws

> ros2 topic list
# 用于和controller通信的topic
/diff_drive_base_controller/cmd_vel_unstamped
/diff_drive_base_controller/odom
/diff_drive_base_controller/transition_event

/dynamic_joint_states
/joint_state_broadcaster/transition_event
/joint_states

/parameter_events
/performance_metrics

/robot_description

/rosout
/tf
/tf_static
```

### service

```bash
trantor ➜ ~/project/demo_ws

> ros2 service list
/apply_joint_effort
/apply_link_wrench
/clear_joint_efforts
/clear_link_wrenches

/delete_entity
/get_model_list
/pause_physics
/reset_simulation
/reset_world
/spawn_entity
/unpause_physics

/controller_manager/configure_controller
/controller_manager/describe_parameters
/controller_manager/get_parameter_types
/controller_manager/get_parameters
/controller_manager/list_controller_types
/controller_manager/list_controllers
/controller_manager/list_hardware_components
/controller_manager/list_hardware_interfaces
/controller_manager/list_parameters
/controller_manager/load_controller
/controller_manager/reload_controller_libraries
/controller_manager/set_hardware_component_state
/controller_manager/set_parameters
/controller_manager/set_parameters_atomically
/controller_manager/switch_controller
/controller_manager/unload_controller

/diff_drive_base_controller/describe_parameters
/diff_drive_base_controller/get_parameter_types
/diff_drive_base_controller/get_parameters
/diff_drive_base_controller/list_parameters
/diff_drive_base_controller/set_parameters
/diff_drive_base_controller/set_parameters_atomically

/gazebo/describe_parameters
/gazebo/get_parameter_types
/gazebo/get_parameters
/gazebo/list_parameters
/gazebo/set_parameters
/gazebo/set_parameters_atomically

/gazebo_ros2_control/describe_parameters
/gazebo_ros2_control/get_parameter_types
/gazebo_ros2_control/get_parameters
/gazebo_ros2_control/list_parameters
/gazebo_ros2_control/set_parameters
/gazebo_ros2_control/set_parameters_atomically

/joint_state_broadcaster/describe_parameters
/joint_state_broadcaster/get_parameter_types
/joint_state_broadcaster/get_parameters
/joint_state_broadcaster/list_parameters
/joint_state_broadcaster/set_parameters
/joint_state_broadcaster/set_parameters_atomically

/robot_state_publisher/describe_parameters
/robot_state_publisher/get_parameter_types
/robot_state_publisher/get_parameters
/robot_state_publisher/list_parameters
/robot_state_publisher/set_parameters
/robot_state_publisher/set_parameters_atomically
```
