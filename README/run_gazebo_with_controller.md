---
data: 2024-07-24 22:15:04
---

- [controller.yaml](#controlleryaml)
  - [launch 文件加载 与 controller.yaml 配置](#launch-文件加载-与-controlleryaml-配置)
- [run gazebo with controller](#run-gazebo-with-controller)

## controller.yaml

### launch 文件加载 与 controller.yaml 配置

在 controller.yaml 文件中指定 controller 类型：

```yaml
# FL Controller
FL_hip_controller:
  type: unitree_joint_controller/UnitreeJointController
FL_thigh_controller:
  type: unitree_joint_controller/UnitreeJointController
FL_calf_controller:
  type: unitree_joint_controller/UnitreeJointController
```

同时在 `FL_hip_controller` 字段下指定 `joint_name` 字段，并使用数组形式：

```yaml
FL_hip_controller:
  ros__parameters:
    update_rate: 10
    desired_cmd_timeout: 0.5
    joint_name: [FL_hip_joint]
    pid:
      p: 100.0
      i: 0.0
      d: 5.0
```

数组形式的指定是与 "unitree_joint_controller_parameter.yaml" 文件中的定义相互关联。这里也是复用了 "unitree_position_controller" package 中的惯例。

同时，这部分指定的 yaml，会通过 "generate_parameter_library" 包生成 paraments 代码用于与 controller_manager 相互关联。

```yaml
joint_name: {
    type: string_array, #
    default_value: [],
    description: "unitree motor joint",
  }
```

## run gazebo with controller

初步使用 `ros2 launch robot_description gazebo.launch.py` 加载 “模型”，“控制器” 等组件，需要再详细分析中的 log，完善整体的流程、细节。

```log
> ros2 launch robot_description gazebo.launch.py

[INFO] [launch]: All log files can be found below /home/trantor/.ros/log/2024-07-24-22-06-10-194640-vmware-35063
[INFO] [launch]: Default logging verbosity is set to INFO

[WARNING] [launch_ros.actions.node]: Parameter file path is not a file: robot_description

[INFO] [gzserver-1]: process started with pid [35065]
[INFO] [gzclient-2]: process started with pid [35067]
[INFO] [robot_state_publisher-3]: process started with pid [35069]
[INFO] [ros2_control_node-4]: process started with pid [35071]
[INFO] [spawner-5]: process started with pid [35073]

[robot_state_publisher-3] [INFO] [1721829971.763202128] [robot_state_publisher]: got segment FL_calf
[robot_state_publisher-3] [INFO] [1721829971.763397421] [robot_state_publisher]: got segment FL_foot
[robot_state_publisher-3] [INFO] [1721829971.763411805] [robot_state_publisher]: got segment FL_hip
[robot_state_publisher-3] [INFO] [1721829971.763420282] [robot_state_publisher]: got segment FL_thigh
[robot_state_publisher-3] [INFO] [1721829971.763428073] [robot_state_publisher]: got segment FL_thigh_shoulder
[robot_state_publisher-3] [INFO] [1721829971.763435606] [robot_state_publisher]: got segment FR_calf
[robot_state_publisher-3] [INFO] [1721829971.763442877] [robot_state_publisher]: got segment FR_foot
[robot_state_publisher-3] [INFO] [1721829971.763450058] [robot_state_publisher]: got segment FR_hip
[robot_state_publisher-3] [INFO] [1721829971.763457779] [robot_state_publisher]: got segment FR_thigh
[robot_state_publisher-3] [INFO] [1721829971.763464977] [robot_state_publisher]: got segment FR_thigh_shoulder
[robot_state_publisher-3] [INFO] [1721829971.763472553] [robot_state_publisher]: got segment RL_calf
[robot_state_publisher-3] [INFO] [1721829971.763479766] [robot_state_publisher]: got segment RL_foot
[robot_state_publisher-3] [INFO] [1721829971.763486687] [robot_state_publisher]: got segment RL_hip
[robot_state_publisher-3] [INFO] [1721829971.763493761] [robot_state_publisher]: got segment RL_thigh
[robot_state_publisher-3] [INFO] [1721829971.763500934] [robot_state_publisher]: got segment RL_thigh_shoulder
[robot_state_publisher-3] [INFO] [1721829971.763508166] [robot_state_publisher]: got segment RR_calf
[robot_state_publisher-3] [INFO] [1721829971.763515423] [robot_state_publisher]: got segment RR_foot
[robot_state_publisher-3] [INFO] [1721829971.763522514] [robot_state_publisher]: got segment RR_hip
[robot_state_publisher-3] [INFO] [1721829971.763529632] [robot_state_publisher]: got segment RR_thigh
[robot_state_publisher-3] [INFO] [1721829971.763536619] [robot_state_publisher]: got segment RR_thigh_shoulder
[robot_state_publisher-3] [INFO] [1721829971.763543665] [robot_state_publisher]: got segment base
[robot_state_publisher-3] [INFO] [1721829971.763550770] [robot_state_publisher]: got segment imu_link
[robot_state_publisher-3] [INFO] [1721829971.763557762] [robot_state_publisher]: got segment trunk

[ros2_control_node-4] [INFO] [1721829971.766655762] [controller_manager]: Subscribing to '~/robot_description' topic for robot description file.
[ros2_control_node-4] [INFO] [1721829971.769099034] [controller_manager]: update rate is 10 Hz
[ros2_control_node-4] [INFO] [1721829971.769391024] [controller_manager]: RT kernel is recommended for better performance

[gzclient-2] context mismatch in svga_surface_destroy
[gzclient-2] context mismatch in svga_surface_destroy

[spawner-5] [INFO] [1721829974.030402763] [spawner_joint_state_broadcaster]: Waiting for '/controller_manager' services to be available
[spawner-5] [INFO] [1721829976.041468778] [spawner_joint_state_broadcaster]: Waiting for '/controller_manager' services to be available

[INFO] [spawn_entity.py-6]: process started with pid [35249]
[spawn_entity.py-6] [INFO] [1721829977.010867816] [spawn_entity]: Spawn Entity started
[spawn_entity.py-6] [INFO] [1721829977.011403712] [spawn_entity]: Loading entity published on topic robot_description
[spawn_entity.py-6] [INFO] [1721829977.013849136] [spawn_entity]: Waiting for entity xml on robot_description
[spawn_entity.py-6] [INFO] [1721829977.016425743] [spawn_entity]: Waiting for service /spawn_entity, timeout = 30
[spawn_entity.py-6] [INFO] [1721829977.016880353] [spawn_entity]: Waiting for service /spawn_entity
[spawn_entity.py-6] [INFO] [1721829977.022426465] [spawn_entity]: Calling service /spawn_entity
[spawn_entity.py-6] [INFO] [1721829977.335237195] [spawn_entity]: Spawn status: SpawnEntity: Successfully spawned entity [robot_a1]

[gzserver-1] [INFO] [1721829977.378646465] [gazebo_ros2_control]: Loading gazebo_ros2_control plugin
[gzserver-1] [INFO] [1721829977.386860428] [gazebo_ros2_control]: Starting gazebo_ros2_control plugin in namespace: /
[gzserver-1] [INFO] [1721829977.387163715] [gazebo_ros2_control]: Starting gazebo_ros2_control plugin in ros 2 node: gazebo_ros2_control
[gzserver-1] [INFO] [1721829977.391282202] [gazebo_ros2_control]: connected to service!! robot_state_publisher
[gzserver-1] [INFO] [1721829977.393537448] [gazebo_ros2_control]: Received urdf from param server, parsing...
[gzserver-1] [INFO] [1721829977.393939337] [gazebo_ros2_control]: Loading parameter files /home/trantor/project/model_ws/install/robot_description/share/robot_description/config/controller.yaml

[gzserver-1] [INFO] [1721829977.435565854] [gazebo_ros2_control]: Loading joint: FL_hip_joint
[gzserver-1] [INFO] [1721829977.435734592] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1721829977.435784556] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1721829977.436236346] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.436415882] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1721829977.436487760] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437014992] [gazebo_ros2_control]: Loading joint: FL_thigh_joint
[gzserver-1] [INFO] [1721829977.437033031] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1721829977.437041922] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1721829977.437053410] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437061886] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1721829977.437076218] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437249633] [gazebo_ros2_control]: Loading joint: FL_calf_joint
[gzserver-1] [INFO] [1721829977.437265599] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1721829977.437273837] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1721829977.437284375] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437292875] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1721829977.437328536] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437342840] [gazebo_ros2_control]: Loading joint: FR_hip_joint
[gzserver-1] [INFO] [1721829977.437352816] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1721829977.437361126] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1721829977.437369614] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437378074] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1721829977.437407188] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437419875] [gazebo_ros2_control]: Loading joint: FR_thigh_joint
[gzserver-1] [INFO] [1721829977.437429659] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1721829977.437437326] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1721829977.437447471] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437457319] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1721829977.437467893] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437480524] [gazebo_ros2_control]: Loading joint: FR_calf_joint
[gzserver-1] [INFO] [1721829977.437489932] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1721829977.437497530] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1721829977.437505483] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437513396] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1721829977.437523108] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437533715] [gazebo_ros2_control]: Loading joint: RL_hip_joint
[gzserver-1] [INFO] [1721829977.437542496] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1721829977.437550020] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1721829977.437557929] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437565819] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1721829977.437579179] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437589869] [gazebo_ros2_control]: Loading joint: RL_thigh_joint
[gzserver-1] [INFO] [1721829977.437598594] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1721829977.437606318] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1721829977.437614202] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437622149] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1721829977.437631942] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437643916] [gazebo_ros2_control]: Loading joint: RL_calf_joint
[gzserver-1] [INFO] [1721829977.437652904] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1721829977.437660260] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1721829977.437694712] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437706749] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1721829977.437718634] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437733455] [gazebo_ros2_control]: Loading joint: RR_hip_joint
[gzserver-1] [INFO] [1721829977.437743505] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1721829977.437751292] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1721829977.437759394] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437767456] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1721829977.437781056] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437793671] [gazebo_ros2_control]: Loading joint: RR_thigh_joint
[gzserver-1] [INFO] [1721829977.437802758] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1721829977.437810462] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1721829977.437818605] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437826648] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1721829977.437836713] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437847790] [gazebo_ros2_control]: Loading joint: RR_calf_joint
[gzserver-1] [INFO] [1721829977.437856652] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1721829977.437864187] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1721829977.437872091] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1721829977.437880023] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1721829977.437889367] [gazebo_ros2_control]: 		 velocity

[gzserver-1] [INFO] [1721829977.438641468] [resource_manager]: Initialize hardware 'GazeboSystem'
[gzserver-1] [INFO] [1721829977.440408041] [resource_manager]: Successful initialization of hardware 'GazeboSystem'
[gzserver-1] [INFO] [1721829977.448053748] [resource_manager]: 'configure' hardware 'GazeboSystem'
[gzserver-1] [INFO] [1721829977.448230433] [resource_manager]: Successful 'configure' of hardware 'GazeboSystem'
[gzserver-1] [INFO] [1721829977.448376948] [resource_manager]: 'activate' hardware 'GazeboSystem'
[gzserver-1] [INFO] [1721829977.448386952] [resource_manager]: Successful 'activate' of hardware 'GazeboSystem'

[gzserver-1] [INFO] [1721829977.448915288] [gazebo_ros2_control]: Loading controller_manager
[gzserver-1] [WARN] [1721829977.510199015] [gazebo_ros2_control]:  Desired controller update period (0.1 s) is slower than the gazebo simulation period (0.001 s).
[gzserver-1] [INFO] [1721829977.511765562] [gazebo_ros2_control]: Loaded gazebo_ros2_control.

[INFO] [spawn_entity.py-6]: process has finished cleanly [pid 35249]
[gzserver-1] [INFO] [1721829977.661948678] [controller_manager]: Loading controller 'joint_state_broadcaster'
[spawner-5] [INFO] [1721829977.713120326] [spawner_joint_state_broadcaster]: Loaded joint_state_broadcaster
[gzserver-1] [INFO] [1721829977.715850992] [controller_manager]: Configuring controller 'joint_state_broadcaster'
[gzserver-1] [INFO] [1721829977.717491387] [joint_state_broadcaster]: 'joints' or 'interfaces' parameter is empty. All available state interfaces will be published
[spawner-5] [INFO] [1721829978.020317365] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[INFO] [spawner-5]: process has finished cleanly [pid 35073]

1 [INFO] [spawner-7]: process started with pid [35274]
2 [INFO] [spawner-8]: process started with pid [35276]
3 [INFO] [spawner-9]: process started with pid [35278]
4 [INFO] [spawner-10]: process started with pid [35280]
5 [INFO] [spawner-11]: process started with pid [35282]
6 [INFO] [spawner-12]: process started with pid [35284]
7 [INFO] [spawner-13]: process started with pid [35286]
8 [INFO] [spawner-14]: process started with pid [35288]
9 [INFO] [spawner-15]: process started with pid [35290]
10 [INFO] [spawner-16]: process started with pid [35292]
11 [INFO] [spawner-17]: process started with pid [35294]
12 [INFO] [spawner-18]: process started with pid [35296]

[gzserver-1] [INFO] [1721829982.396641071] [controller_manager]: Loading controller 'FR_thigh_controller'
[gzserver-1] [INFO] [1721829982.627687644] [controller_manager]: Loading controller 'RR_calf_controller'
[gzserver-1] [INFO] [1721829982.717113661] [controller_manager]: Loading controller 'FL_thigh_controller'
[gzserver-1] [INFO] [1721829982.927244915] [controller_manager]: Loading controller 'RR_thigh_controller'
[gzserver-1] [INFO] [1721829983.131535447] [controller_manager]: Loading controller 'RL_hip_controller'
[gzserver-1] [INFO] [1721829983.638937446] [controller_manager]: Loading controller 'FL_calf_controller'
[gzserver-1] [INFO] [1721829984.048775615] [controller_manager]: Loading controller 'RL_thigh_controller'
[gzserver-1] [INFO] [1721829984.554440636] [controller_manager]: Loading controller 'FL_hip_controller'
[gzserver-1] [INFO] [1721829984.961303701] [controller_manager]: Loading controller 'FR_hip_controller'
[gzserver-1] [INFO] [1721829985.502224509] [controller_manager]: Loading controller 'RR_hip_controller'
[gzserver-1] [INFO] [1721829985.869214604] [controller_manager]: Loading controller 'FR_calf_controller'
[gzserver-1] [INFO] [1721829986.425357378] [controller_manager]: Loading controller 'RL_calf_controller'

[gzserver-1] [INFO] [1721829982.825638848] [controller_manager]: Configuring controller 'FR_thigh_controller'
[gzserver-1] [INFO] [1721829983.030274219] [controller_manager]: Configuring controller 'RR_calf_controller'
[gzserver-1] [INFO] [1721829983.339256744] [controller_manager]: Configuring controller 'FL_thigh_controller'
[gzserver-1] [INFO] [1721829983.743623015] [controller_manager]: Configuring controller 'RR_thigh_controller'
[gzserver-1] [INFO] [1721829984.293123174] [controller_manager]: Configuring controller 'RL_hip_controller'
[gzserver-1] [INFO] [1721829984.657919735] [controller_manager]: Configuring controller 'FL_calf_controller'
[gzserver-1] [INFO] [1721829985.066905591] [controller_manager]: Configuring controller 'RL_thigh_controller'
[gzserver-1] [INFO] [1721829985.579304165] [controller_manager]: Configuring controller 'FL_hip_controller'
[gzserver-1] [INFO] [1721829985.974487508] [controller_manager]: Configuring controller 'FR_hip_controller'
[gzserver-1] [INFO] [1721829986.526903402] [controller_manager]: Configuring controller 'RR_hip_controller'
[gzserver-1] [INFO] [1721829986.831574730] [controller_manager]: Configuring controller 'FR_calf_controller'
[gzserver-1] [INFO] [1721829987.291037027] [controller_manager]: Configuring controller 'RL_calf_controller'

[gzserver-1] [WARN] [1721829983.533109348] [FR_thigh_controller]: Error occurred while doing error handling.
[gzserver-1] [WARN] [1721829983.944671633] [RR_calf_controller]: Error occurred while doing error handling.
[gzserver-1] [WARN] [1721829984.451705161] [FL_thigh_controller]: Error occurred while doing error handling.
[gzserver-1] [WARN] [1721829984.858519753] [RR_thigh_controller]: Error occurred while doing error handling.
[gzserver-1] [WARN] [1721829985.463842848] [RL_hip_controller]: Error occurred while doing error handling.
[gzserver-1] [WARN] [1721829985.766925183] [FL_calf_controller]: Error occurred while doing error handling.
[gzserver-1] [WARN] [1721829986.174248410] [RL_thigh_controller]: Error occurred while doing error handling.
[gzserver-1] [WARN] [1721829986.728338088] [FL_hip_controller]: Error occurred while doing error handling.
[gzserver-1] [WARN] [1721829987.032666184] [FR_hip_controller]: Error occurred while doing error handling.
[gzserver-1] [WARN] [1721829987.396024303] [RR_hip_controller]: Error occurred while doing error handling.
[gzserver-1] [WARN] [1721829987.584973773] [FR_calf_controller]: Error occurred while doing error handling.
[gzserver-1] [WARN] [1721829987.784717500] [RL_calf_controller]: Error occurred while doing error handling.

[gzserver-1] [ERROR] [1721829983.533005398] [unitree_motor_controller]: no joint names specified
[gzserver-1] [ERROR] [1721829983.944609228] [unitree_motor_controller]: no joint names specified
[gzserver-1] [ERROR] [1721829984.451582798] [unitree_motor_controller]: no joint names specified
[gzserver-1] [ERROR] [1721829984.858281994] [unitree_motor_controller]: no joint names specified
[gzserver-1] [ERROR] [1721829985.463760347] [unitree_motor_controller]: no joint names specified
[gzserver-1] [ERROR] [1721829985.766850954] [unitree_motor_controller]: no joint names specified
[gzserver-1] [ERROR] [1721829986.174162410] [unitree_motor_controller]: no joint names specified
[gzserver-1] [ERROR] [1721829986.728254459] [unitree_motor_controller]: no joint names specified
[gzserver-1] [ERROR] [1721829987.032560757] [unitree_motor_controller]: no joint names specified
[gzserver-1] [ERROR] [1721829987.395731842] [unitree_motor_controller]: no joint names specified
[gzserver-1] [ERROR] [1721829987.584906287] [unitree_motor_controller]: no joint names specified
[gzserver-1] [ERROR] [1721829987.784155954] [unitree_motor_controller]: no joint names specified

[gzserver-1] [ERROR] [1721829983.535053626] [controller_manager]: After activation, controller 'FR_thigh_controller' is in state 'unconfigured' (1), expected 'active' (3).
[gzserver-1] [ERROR] [1721829983.947140683] [controller_manager]: After activation, controller 'RR_calf_controller' is in state 'unconfigured' (1), expected 'active' (3).
[gzserver-1] [ERROR] [1721829984.452954189] [controller_manager]: After activation, controller 'FL_thigh_controller' is in state 'unconfigured' (1), expected 'active' (3).
[gzserver-1] [ERROR] [1721829984.859413141] [controller_manager]: After activation, controller 'RR_thigh_controller' is in state 'unconfigured' (1), expected 'active' (3).
[gzserver-1] [ERROR] [1721829985.465037564] [controller_manager]: After activation, controller 'RL_hip_controller' is in state 'unconfigured' (1), expected 'active' (3).
[gzserver-1] [ERROR] [1721829985.768004858] [controller_manager]: After activation, controller 'FL_calf_controller' is in state 'unconfigured' (1), expected 'active' (3).
[gzserver-1] [ERROR] [1721829986.322513129] [controller_manager]: After activation, controller 'RL_thigh_controller' is in state 'unconfigured' (1), expected 'active' (3).
[gzserver-1] [ERROR] [1721829986.730515902] [controller_manager]: After activation, controller 'FL_hip_controller' is in state 'unconfigured' (1), expected 'active' (3).
[gzserver-1] [ERROR] [1721829987.033856551] [controller_manager]: After activation, controller 'FR_hip_controller' is in state 'unconfigured' (1), expected 'active' (3).
[gzserver-1] [ERROR] [1721829987.397772453] [controller_manager]: After activation, controller 'RR_hip_controller' is in state 'unconfigured' (1), expected 'active' (3).
[gzserver-1] [ERROR] [1721829987.586154650] [controller_manager]: After activation, controller 'FR_calf_controller' is in state 'unconfigured' (1), expected 'active' (3).
[gzserver-1] [ERROR] [1721829987.785847562] [controller_manager]: After activation, controller 'RL_calf_controller' is in state 'unconfigured' (1), expected 'active' (3).

[spawner-7] [INFO] [1721829984.678046012] [spawner_FL_hip_controller]: Loaded FL_hip_controller
[spawner-8] [INFO] [1721829982.855409780] [spawner_FL_thigh_controller]: Loaded FL_thigh_controller
[spawner-9] [INFO] [1721829983.763452770] [spawner_FL_calf_controller]: Loaded FL_calf_controller
[spawner-10] [INFO] [1721829985.090068549] [spawner_FR_hip_controller]: Loaded FR_hip_controller
[spawner-11] [INFO] [1721829982.658487660] [spawner_FR_thigh_controller]: Loaded FR_thigh_controller
[spawner-12] [INFO] [1721829986.001143032] [spawner_FR_calf_controller]: Loaded FR_calf_controller
[spawner-13] [INFO] [1721829983.362628831] [spawner_RL_hip_controller]: Loaded RL_hip_controller
[spawner-14] [INFO] [1721829984.149597936] [spawner_RL_thigh_controller]: Loaded RL_thigh_controller
[spawner-17] [INFO] [1721829983.052114535] [spawner_RR_thigh_controller]: Loaded RR_thigh_controller
[spawner-15] [INFO] [1721829986.547508697] [spawner_RL_calf_controller]: Loaded RL_calf_controller
[spawner-16] [INFO] [1721829985.580597577] [spawner_RR_hip_controller]: Loaded RR_hip_controller
[spawner-18] [INFO] [1721829982.773812268] [spawner_RR_calf_controller]: Loaded RR_calf_controller

[spawner-7] [INFO] [1721829986.832284821] [spawner_FL_hip_controller]: Configured and activated FL_hip_controller
[spawner-8] [INFO] [1721829984.553465949] [spawner_FL_thigh_controller]: Configured and activated FL_thigh_controller
[spawner-9] [INFO] [1721829985.869993328] [spawner_FL_calf_controller]: Configured and activated FL_calf_controller
[spawner-10] [INFO] [1721829987.291530091] [spawner_FR_hip_controller]: Configured and activated FR_hip_controller
[spawner-11] [INFO] [1721829983.637038366] [spawner_FR_thigh_controller]: Configured and activated FR_thigh_controller
[spawner-12] [INFO] [1721829987.686141922] [spawner_FR_calf_controller]: Configured and activated FR_calf_controller
[spawner-13] [INFO] [1721829985.502724343] [spawner_RL_hip_controller]: Configured and activated RL_hip_controller
[spawner-14] [INFO] [1721829986.425340151] [spawner_RL_thigh_controller]: Configured and activated RL_thigh_controller
[spawner-15] [INFO] [1721829987.887420681] [spawner_RL_calf_controller]: Configured and activated RL_calf_controller
[spawner-16] [INFO] [1721829987.488517477] [spawner_RR_hip_controller]: Configured and activated RR_hip_controller
[spawner-17] [INFO] [1721829984.960613436] [spawner_RR_thigh_controller]: Configured and activated RR_thigh_controller
[spawner-18] [INFO] [1721829984.048407171] [spawner_RR_calf_controller]: Configured and activated RR_calf_controller

[INFO] [spawner-7]: process has finished cleanly [pid 35274]
[INFO] [spawner-8]: process has finished cleanly [pid 35276]
[INFO] [spawner-9]: process has finished cleanly [pid 35278]
[INFO] [spawner-10]: process has finished cleanly [pid 35280]
[INFO] [spawner-11]: process has finished cleanly [pid 35282]
[INFO] [spawner-12]: process has finished cleanly [pid 35284]
[INFO] [spawner-13]: process has finished cleanly [pid 35286]
[INFO] [spawner-14]: process has finished cleanly [pid 35288]
[INFO] [spawner-15]: process has finished cleanly [pid 35290]
[INFO] [spawner-16]: process has finished cleanly [pid 35292]
[INFO] [spawner-17]: process has finished cleanly [pid 35294]
[INFO] [spawner-18]: process has finished cleanly [pid 35296]

```
