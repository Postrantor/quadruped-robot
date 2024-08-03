---
date: 2024-07-21 22:10:40
---

### `gazebo_ros_state`

通过加载插件 `libgazebo_ros_state.so`，gazebo 会提供如下的内容：

- service: `/get_entity_state`
- service: `/set_entity_state`
- topic: `/link_states`
- topic: `/model_states`

加载插件的方式：

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
    <world name="default">
        <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
            <ros>
                <namespace>/</namespace>
            </ros>
            <update_rate>1.0</update_rate>
        </plugin>
    </world>
</sdf>
```

#### gazebo->pub `/link_states`

```bash
> ros2 topic info -v /link_states
Type: gazebo_msgs/msg/LinkStates

Publisher count: 1

Node name: gazebo_ros_state
Node namespace: /
Topic type: gazebo_msgs/msg/LinkStates
Endpoint type: PUBLISHER
GID: 01.10.d0.ac.06.2f.84.5a.87.32.18.77.00.00.43.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (1)
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 0

trantor@VMWARE ➜ ~
> ros2 interface show gazebo_msgs/msg/LinkStates
# broadcast all link states in world frame
string[] name                 # link names
geometry_msgs/Pose[] pose     # desired pose in world frame
	Point position
		float64 x
		float64 y
		float64 z
	Quaternion orientation
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1
geometry_msgs/Twist[] twist   # desired twist in world frame
	Vector3  linear
		float64 x
		float64 y
		float64 z
	Vector3  angular
		float64 x
		float64 y
		float64 z

trantor@VMWARE ➜ ~
> ros2 topic echo /link_states
name:
- ground_plane::link
- robot_a1::base
- robot_a1::FL_hip
- robot_a1::FL_thigh
- robot_a1::FL_calf
- robot_a1::FR_hip
- robot_a1::FR_thigh
- robot_a1::FR_calf
- robot_a1::RL_hip
- robot_a1::RL_thigh
- robot_a1::RL_calf
- robot_a1::RR_hip
- robot_a1::RR_thigh
- robot_a1::RR_calf
pose:
- position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
- position:
    x: -0.35557669806701764
    y: -1.6278860441879226
    z: 0.05692905471205958
  orientation:
    x: -6.137232671557229e-05
    y: 2.8644921299596094e-05
    z: -0.7832812127484844
    w: -0.6216675455323124
... ...
```

#### gazebo->pub `model_states`

```bash
> ros2 topic info -v /model_states
Type: gazebo_msgs/msg/ModelStates

Publisher count: 1

Node name: gazebo_ros_state
Node namespace: /
Topic type: gazebo_msgs/msg/ModelStates
Endpoint type: PUBLISHER
GID: 01.10.d0.ac.06.2f.84.5a.87.32.18.77.00.00.42.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (1)
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 0

trantor@VMWARE ➜ ~
> ros2 interface show gazebo_msgs/msg/ModelStates
# broadcast all model states in world frame
string[] name                 # model names
geometry_msgs/Pose[] pose     # desired pose in world frame
	Point position
		float64 x
		float64 y
		float64 z
	Quaternion orientation
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1
geometry_msgs/Twist[] twist   # desired twist in world frame
	Vector3  linear
		float64 x
		float64 y
		float64 z
	Vector3  angular
		float64 x
		float64 y
		float64 z
```

#### `set_entity_state`

```bash
> ros2 service type /set_entity_state
gazebo_msgs/srv/SetEntityState
trantor@VMWARE ➜ ~
> ros2 interface show gazebo_msgs/srv/SetEntityState
gazebo_msgs/EntityState state   # Entity state to set to.
	string name                 #
	                            # An entity can be a model, link, collision, light, etc.
	                            # Be sure to use gazebo scoped naming notation (e.g. [model_name::link_name])
	geometry_msgs/Pose pose     #
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Twist twist   #
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	string reference_frame      #
	                            # Leaving empty or "world" defaults to inertial world frame.
                                # Be sure to fill all fields, values of zero have meaning.
---
bool success                    # Return true if setting state was successful.
```

```bash
> ros2 topic echo /link_states
name:
- ground_plane::link
- robot_a1::base
- robot_a1::FL_hip
- robot_a1::FL_thigh
- robot_a1::FL_calf
- robot_a1::FR_hip
- robot_a1::FR_thigh
- robot_a1::FR_calf
- robot_a1::RL_hip
- robot_a1::RL_thigh
- robot_a1::RL_calf
- robot_a1::RR_hip
- robot_a1::RR_thigh
- robot_a1::RR_calf
```

### service `set_model_configuration`

对于 `set_model_configuration` 服务，在 ros2 中未支持，但可以通过上述 4 个消息进行替换

```bash
> ros2 interface show gazebo_msgs/srv/SetModelConfiguration
# Set joint positions for a model
string model_name           # model to set state
string urdf_param_name      # UNUSED

string[] joint_names        # list of joints to set positions.  if joint is not listed here, preserve current position.
float64[] joint_positions   # set to this position.
---
bool success                # return true if setting state successful
string status_message       # comments if available
```

### start

```bash
ros2 launch robot_description gazebo.launch.py --debug
ros2 control list_controllers -v
```

###

尝试通过命令行方式发送控制指令可能会有问题？
可能是以为数据类型不完全匹配的问题，需要看一下 controller 的细节。

```bash
ros2 topic pub --rate 1 /FL_calf_controller/desired_state geometry_msgs/msg/TwistStamped "{
  twist: {
    linear: {x: 0.7, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 1.0},
  }
}"

ros2 topic pub --rate 1 /FL_calf_controller/desired_cmd unitree_msgs/msg/MotorCmd "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'frame_id_string'
  },
  hex_len: 17,
  id: 0,
  mode: 10,
  tau: 2.0,
  dq: 2.0,
  q: 15.0,
  k_q: 5.0,
  k_dq: 5.0
}"
```

### gazebo 调试

默认起一个 gazebo server 等价于如下指令，会加载一些 so：

```bash
> gzserver /home/trantor/project/model_ws/install/robot_description/share/robot_description/worlds/earth.world -slibgazebo_ros_init.so -slibgazebo_ros_factory.so -slibgazebo_ros_force_system.so

> ros2 service list
/spawn_entity
/get_model_list
/apply_joint_effort
/apply_link_wrench
/clear_joint_efforts
/clear_link_wrenches
/delete_entity
/reset_simulation
/reset_world
/pause_physics
/unpause_physics

/gazebo/describe_parameters
/gazebo/get_parameter_types
/gazebo/get_parameters
/gazebo/list_parameters
/gazebo/set_parameters
/gazebo/set_parameters_atomically
```

### simuliation 文件结构

- "gazebo_plugins": 用于模拟传感器等逐渐的第三方插件，由 gazebo 官方提供；
- "gazebo_ros":
- "qr_gazebo": 支持 ROS1
  - launcher: 提供 launcher 文件，加载相应组件；
  - controller_manager: 提供控制不同 controller 的组件；
  - plugin: 提供辅助插件，用于在 xpp 中绘制受力关系；
- "robot_description": 基于 ROS2 的功能包，用于将模型加载到 gazebo 中；
- 'xpp': 支持 ROS1，用于在 rviz 界面中绘制足端受力情况，需要 gazebo plugin 相应插件支持，由 "qr_gazebo" 插件提供；

### gazebo_ros_pkgs

用于与 Gazebo 接口的 ROS 2 包集包含在名为 gazebo_ros_pkgs 的元包中。在继续此处之前，请参阅 ROS 2 概述以获取背景信息。

[gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2)
[demo](https://github.com/ros-simulation/gazebo_ros_demos/tree/foxy)
[tutorials](https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)

支持 ROS2 开发，包含的功能包：

- "gazebo_ros_pkgs": 元功能包；
- "gazebo_dev": gazebo 依赖功能包，用于编译的功能包；

- "gazebo_msgs" : interfaces 定义；

- "gazebo_plugins" 用于控制 gazebo 中对象的第三方插件，如 IMU 传感器等，包示例的括机器人主体，有 diff_drive_robot
  其中一些 IMU/Position 等组件还是可以用的；
- "gazebo_ros": 用于 ROS2 <-> Gazebo 交互层，编译的产物为 so，通过插件的方式集成；
- "gazebo_ros_control": 依赖功能包，这是一个 ROS 2 软件包，用于将 ros2_control 控制器架构与 Gazebo Classic 模拟器集成；
  该功能包使用的 API 与 gazebo_ros 相同，应该是等价关系；

关于该功能包的迁移，可以参考：

![](simulation/gazebo_demos/README.md)
