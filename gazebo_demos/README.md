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

通过 `ros2 service type <service_name>` 查看对应服务类型，按照相同的类型对比 example 实例程序中的 srv

#### ROS2 --> ROS1 迁移相关文档

在 ROS1 -> ROS2 的过程中有一些服务被拆分了，所以这部分代码在移植过程中也要注意，以下是 gazebo_ros 包中的一些相关讨论和发布内容：

[ros2 gazebo server list](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/779)
[gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_ros/CMakeLists.txt)

[gazebo_ros_api_plugin](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-gazebo_ros_api_plugin)
[Spawn and delete](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Spawn-and-delete)
[Set_model_configuration service ported to ROS 2](https://discourse.ros.org/t/set-model-configuration-service-ported-to-ros-2-gazebo-ros/32995)

以下链接是 ROS1 中 gazebo_ros 提供的服务列表示例：

![](ascend/model-control/README.md)

#### 目前使用的仿真中默认包含的 service

```bash
trantor ➜ ~/project/demo_ws

> ros2 service list
## support by `gazebo_ros_force_system`
/apply_joint_effort
  # > gazebo_msgs/srv/ApplyJointEffort
/apply_link_wrench
  # > gazebo_msgs/srv/ApplyLinkWrench
/clear_joint_efforts
  # > gazebo_msgs/srv/JointRequest
/clear_link_wrenches
  # > gazebo_msgs/srv/LinkRequest

## support by `gazebo_ros_factory`
# [ROS 2 Migration: Spawn and delete](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Spawn-and-delete)
/delete_entity
  # > gazebo_msgs/srv/DeleteEntity
/spawn_entity
  # > gazebo_msgs/srv/SpawnEntity

/get_model_list
  # > gazebo_msgs/srv/GetModelList

## support by `gazebo_ros_init`
/reset_simulation
  # > std_srvs/srv/Empty
/reset_world
  # > std_srvs/srv/Empty
/pause_physics
  # > std_srvs/srv/Empty
/unpause_physics
  # > std_srvs/srv/Empty

/controller_manager/describe_parameters
  # > rcl_interfaces/srv/DescribeParameters
/controller_manager/get_parameter_types
  # > rcl_interfaces/srv/GetParameterTypes
/controller_manager/get_parameters
  # > rcl_interfaces/srv/GetParameters
/controller_manager/list_parameters
  # > rcl_interfaces/srv/ListParameters
/controller_manager/set_parameters
  # > rcl_interfaces/srv/SetParameters
/controller_manager/set_parameters_atomically
  # > rcl_interfaces/srv/SetParametersAtomically

/controller_manager/configure_controller
  # > controller_manager_msgs/srv/ConfigureController
/controller_manager/list_controller_types
  # > controller_manager_msgs/srv/ListControllerTypes
/controller_manager/list_controllers
  # > controller_manager_msgs/srv/ListControllers
/controller_manager/list_hardware_components
  # > controller_manager_msgs/srv/ListHardwareComponents
/controller_manager/list_hardware_interfaces
  # > controller_manager_msgs/srv/ListHardwareInterfaces
/controller_manager/load_controller
  # > controller_manager_msgs/srv/LoadController
/controller_manager/reload_controller_libraries
  # > controller_manager_msgs/srv/ReloadControllerLibraries
/controller_manager/set_hardware_component_state
  # > controller_manager_msgs/srv/SetHardwareComponentState
/controller_manager/switch_controller
  # > controller_manager_msgs/srv/SwitchController
/controller_manager/unload_controller
  # > controller_manager_msgs/srv/UnloadController

/diff_drive_base_controller/describe_parameters
  # > rcl_interfaces/srv/DescribeParameters
/diff_drive_base_controller/get_parameter_types
  # > rcl_interfaces/srv/GetParameterTypes
/diff_drive_base_controller/get_parameters
  # > rcl_interfaces/srv/GetParameters
/diff_drive_base_controller/list_parameters
  # > rcl_interfaces/srv/ListParameters
/diff_drive_base_controller/set_parameters
  # > rcl_interfaces/srv/SetParameters
/diff_drive_base_controller/set_parameters_atomically
  # > rcl_interfaces/srv/SetParametersAtomically

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

#### 对比 ROS1 中仿真缺少的 servers

##### 插件 `gazebo_ros_state`

添加这个插件，在 service 中会多出这些

```log
> ros2 node info /gazebo_ros_state
/gazebo_ros_state
  Subscribers:
    /clock: rosgraph_msgs/msg/Clock
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /link_states: gazebo_msgs/msg/LinkStates
    /model_states: gazebo_msgs/msg/ModelStates
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /gazebo_ros_state/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /gazebo_ros_state/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /gazebo_ros_state/get_parameters: rcl_interfaces/srv/GetParameters
    /gazebo_ros_state/list_parameters: rcl_interfaces/srv/ListParameters
    /gazebo_ros_state/set_parameters: rcl_interfaces/srv/SetParameters
    /gazebo_ros_state/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /get_entity_state: gazebo_msgs/srv/GetEntityState
    /set_entity_state: gazebo_msgs/srv/SetEntityState
  Service Clients:

  Action Servers:

  Action Clients:
```

```log
/gazebo_ros_state/describe_parameters
/gazebo_ros_state/get_parameter_types
/gazebo_ros_state/get_parameters
/gazebo_ros_state/list_parameters
/gazebo_ros_state/set_parameters
/gazebo_ros_state/set_parameters_atomically
```

以及一些 topic

```log
[gzserver-1] [INFO] [1722146331.162803978] [gazebo_ros_state]: Publishing states of gazebo models at [/model_states]
[gzserver-1] [INFO] [1722146331.167023273] [gazebo_ros_state]: Publishing states of gazebo links at [/link_states]
```

订阅这两个 topic 的信息可以发现，是 gazebo 在向外反馈机器人的状态信息

```
> ros2 topic info -v /link_states
Type: gazebo_msgs/msg/LinkStates

Publisher count: 1

Node name: gazebo_ros_state
Node namespace: /
Topic type: gazebo_msgs/msg/LinkStates
Endpoint type: PUBLISHER
GID: 01.10.d8.92.48.dd.26.ee.bb.cb.6d.88.00.00.43.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (1)
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 0
```

- service: get_entity_state
- service: set_entity_state
- topic: model_states

这是一个世界插件，其中：

- Provides the get_entity_state ROS service for querying the pose and twist of models, links and lights in simulation.
- Provides the set_entity_state ROS service for setting the pose and twist of models, links and lights in simulation.
- Publishes the model_states ROS topic periodically with the pose and twist of all models in simulation.

- 提供 ROS 服务，get_entity_state 用于查询仿真中模型、链接和灯光的姿态和扭转情况。
- 提供 ROS 服务，set_entity_state 用于在仿真中设置模型、链接和灯光的姿势和扭曲。
- 定期发布 model_states ROS 主题，其中包含仿真中所有模型的姿态和扭转。

See more details on ROS 2 Migration: [Entity states](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Entity-states)

#####

```log
> ros2 launch gazebo_ros2_control_demos robot.launch.py

[INFO] [gzserver-1]: process started with pid [12557]
[INFO] [gzclient-2]: process started with pid [12559]
[INFO] [robot_state_publisher-3]: process started with pid [12561]
[INFO] [spawn_entity.py-4]: process started with pid [12563]
[robot_state_publisher-3] [WARN] [1722148088.366125552] [kdl_parser]: The root link chassis has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[robot_state_publisher-3] [INFO] [1722148088.366513807] [robot_state_publisher]: got segment caster
[robot_state_publisher-3] [INFO] [1722148088.366649290] [robot_state_publisher]: got segment chassis
[robot_state_publisher-3] [INFO] [1722148088.366674503] [robot_state_publisher]: got segment left_wheel
[robot_state_publisher-3] [INFO] [1722148088.366694503] [robot_state_publisher]: got segment right_wheel
[spawn_entity.py-4] [INFO] [1722148088.735728001] [spawn_entity]: Spawn Entity started
[spawn_entity.py-4] [INFO] [1722148088.736174854] [spawn_entity]: Loading entity published on topic robot_description
[spawn_entity.py-4] [INFO] [1722148088.738805119] [spawn_entity]: Waiting for entity xml on robot_description
[spawn_entity.py-4] [INFO] [1722148088.740924785] [spawn_entity]: Waiting for service /spawn_entity, timeout = 30
[spawn_entity.py-4] [INFO] [1722148088.741381347] [spawn_entity]: Waiting for service /spawn_entity
[gzserver-1] [INFO] [1722148089.481451569] [gazebo_ros_state]: Publishing states of gazebo models at [/model_states]
[gzserver-1] [INFO] [1722148089.483798132] [gazebo_ros_state]: Publishing states of gazebo links at [/link_states]
[spawn_entity.py-4] [INFO] [1722148089.497492284] [spawn_entity]: Calling service /spawn_entity
[spawn_entity.py-4] [INFO] [1722148089.750278031] [spawn_entity]: Spawn status: SpawnEntity: Successfully spawned entity [diffdrive]
[gzserver-1] [INFO] [1722148089.791557756] [gazebo_ros2_control]: Loading gazebo_ros2_control plugin
[gzserver-1] [INFO] [1722148089.798216903] [gazebo_ros2_control]: Starting gazebo_ros2_control plugin in namespace: /
[gzserver-1] [INFO] [1722148089.798348759] [gazebo_ros2_control]: Starting gazebo_ros2_control plugin in ros 2 node: gazebo_ros2_control
[gzserver-1] [INFO] [1722148089.802834510] [gazebo_ros2_control]: connected to service!! robot_state_publisher
[gzserver-1] [INFO] [1722148089.804750631] [gazebo_ros2_control]: Received urdf from param server, parsing...
[gzserver-1] [INFO] [1722148089.805260221] [gazebo_ros2_control]: Loading parameter files /home/trantor/project/demo_ws/install/gazebo_ros2_control_demos/share/gazebo_ros2_control_demos/config/controller.yaml
[gzserver-1] [INFO] [1722148089.849426259] [gazebo_ros2_control]: Loading joint: left_wheel_joint
[gzserver-1] [INFO] [1722148089.849700505] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1722148089.849748064] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1722148089.850334195] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1722148089.850641951] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1722148089.850764714] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1722148089.851574653] [gazebo_ros2_control]: Loading joint: right_wheel_joint
[gzserver-1] [INFO] [1722148089.851615453] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1722148089.851645009] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1722148089.851722547] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1722148089.851746077] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1722148089.851783785] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1722148089.853470955] [resource_manager]: Initialize hardware 'GazeboSystem'
[gzserver-1] [INFO] [1722148089.856482494] [resource_manager]: Successful initialization of hardware 'GazeboSystem'
[gzserver-1] [INFO] [1722148089.862613787] [resource_manager]: 'configure' hardware 'GazeboSystem'
[gzserver-1] [INFO] [1722148089.863024462] [resource_manager]: Successful 'configure' of hardware 'GazeboSystem'
[gzserver-1] [INFO] [1722148089.863665043] [resource_manager]: 'activate' hardware 'GazeboSystem'
[gzserver-1] [INFO] [1722148089.863694982] [resource_manager]: Successful 'activate' of hardware 'GazeboSystem'
[gzserver-1] [INFO] [1722148089.865766258] [gazebo_ros2_control]: Loading controller_manager
[gzserver-1] [WARN] [1722148089.947164544] [gazebo_ros2_control]:  Desired controller update period (0.01 s) is slower than the gazebo simulation period (0.001 s).
[gzserver-1] [INFO] [1722148089.948366914] [gazebo_ros2_control]: Loaded gazebo_ros2_control.
[INFO] [spawn_entity.py-4]: process has finished cleanly [pid 12563]
[INFO] [spawner-5]: process started with pid [12669]
[gzserver-1] [INFO] [1722148090.345370213] [controller_manager]: Loading controller 'joint_state_broadcaster'
[spawner-5] [INFO] [1722148090.420926867] [spawner_joint_state_broadcaster]: Loaded joint_state_broadcaster
[gzserver-1] [INFO] [1722148090.424703341] [controller_manager]: Configuring controller 'joint_state_broadcaster'
[gzserver-1] [INFO] [1722148090.426234441] [joint_state_broadcaster]: 'joints' or 'interfaces' parameter is empty. All available state interfaces will be published
[gzserver-1] [INFO] [1722148090.457912768] [controller_manager]: Loading controller 'diff_drive_base_controller'
[spawner-5] [INFO] [1722148090.508866352] [spawner_joint_state_broadcaster]: Loaded diff_drive_base_controller
[gzserver-1] [INFO] [1722148090.510738046] [controller_manager]: Configuring controller 'diff_drive_base_controller'
[spawner-5] [INFO] [1722148090.574776267] [spawner_joint_state_broadcaster]: Configured and activated all the parsed controllers list!
[INFO] [spawner-5]: process has finished cleanly [pid 12669]
[gzclient-2] context mismatch in svga_surface_destroy
[gzclient-2] context mismatch in svga_surface_destroy
```
