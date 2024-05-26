```log
trantor ➜ ~/project/gazebo_ws
> ros2 launch gazebo_ros2_control_demos diff_drive.launch.py
[INFO] [launch]: All log files can be found below /home/trantor/.ros/log/2024-05-26-18-30-20-219748-vmware-32744
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [gzserver-1]: process started with pid [32745]
[INFO] [gzclient-2]: process started with pid [32747]
[INFO] [robot_state_publisher-3]: process started with pid [32749]
[INFO] [spawn_entity.py-4]: process started with pid [32751]

[robot_state_publisher-3] Warning: link 'chassis' material 'orange' undefined.
[robot_state_publisher-3]          at line 84 in ./urdf_parser/src/model.cpp
[robot_state_publisher-3] Warning: link 'chassis' material 'orange' undefined.
[robot_state_publisher-3]          at line 84 in ./urdf_parser/src/model.cpp
[robot_state_publisher-3] Warning: link 'left_wheel' material 'black' undefined.
[robot_state_publisher-3]          at line 84 in ./urdf_parser/src/model.cpp
[robot_state_publisher-3] Warning: link 'left_wheel' material 'black' undefined.
[robot_state_publisher-3]          at line 84 in ./urdf_parser/src/model.cpp
[robot_state_publisher-3] Warning: link 'right_wheel' material 'black' undefined.
[robot_state_publisher-3]          at line 84 in ./urdf_parser/src/model.cpp
[robot_state_publisher-3] Warning: link 'right_wheel' material 'black' undefined.
[robot_state_publisher-3]          at line 84 in ./urdf_parser/src/model.cpp
[robot_state_publisher-3] Warning: link 'caster' material 'white' undefined.
[robot_state_publisher-3]          at line 84 in ./urdf_parser/src/model.cpp
[robot_state_publisher-3] Warning: link 'caster' material 'white' undefined.
[robot_state_publisher-3]          at line 84 in ./urdf_parser/src/model.cpp
[robot_state_publisher-3] [WARN] [1716719420.948663400] [kdl_parser]: The root link chassis has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[robot_state_publisher-3] [INFO] [1716719420.948726856] [robot_state_publisher]: got segment caster
[robot_state_publisher-3] [INFO] [1716719420.948770229] [robot_state_publisher]: got segment chassis
[robot_state_publisher-3] [INFO] [1716719420.948778356] [robot_state_publisher]: got segment left_wheel
[robot_state_publisher-3] [INFO] [1716719420.948784091] [robot_state_publisher]: got segment right_wheel

[spawn_entity.py-4] [INFO] [1716719421.323666518] [spawn_entity]: Spawn Entity started
[spawn_entity.py-4] [INFO] [1716719421.324060699] [spawn_entity]: Loading entity published on topic robot_description
[spawn_entity.py-4] [INFO] [1716719421.325861493] [spawn_entity]: Waiting for entity xml on robot_description
[spawn_entity.py-4] [INFO] [1716719421.328172662] [spawn_entity]: Waiting for service /spawn_entity, timeout = 30
[spawn_entity.py-4] [INFO] [1716719421.328561351] [spawn_entity]: Waiting for service /spawn_entity
[spawn_entity.py-4] [INFO] [1716719423.351636762] [spawn_entity]: Calling service /spawn_entity

[gzserver-1] Warning: link 'chassis' material 'orange' undefined.
[gzserver-1]          at line 84 in ./urdf_parser/src/model.cpp
[gzserver-1] Warning: link 'chassis' material 'orange' undefined.
[gzserver-1]          at line 84 in ./urdf_parser/src/model.cpp
[gzserver-1] Warning: link 'left_wheel' material 'black' undefined.
[gzserver-1]          at line 84 in ./urdf_parser/src/model.cpp
[gzserver-1] Warning: link 'left_wheel' material 'black' undefined.
[gzserver-1]          at line 84 in ./urdf_parser/src/model.cpp
[gzserver-1] Warning: link 'right_wheel' material 'black' undefined.
[gzserver-1]          at line 84 in ./urdf_parser/src/model.cpp
[gzserver-1] Warning: link 'right_wheel' material 'black' undefined.
[gzserver-1]          at line 84 in ./urdf_parser/src/model.cpp
[gzserver-1] Warning: link 'caster' material 'white' undefined.
[gzserver-1]          at line 84 in ./urdf_parser/src/model.cpp
[gzserver-1] Warning: link 'caster' material 'white' undefined.
[gzserver-1]          at line 84 in ./urdf_parser/src/model.cpp

[spawn_entity.py-4] [INFO] [1716719423.470501505] [spawn_entity]: Spawn status: SpawnEntity: Successfully spawned entity [diffbot]

[gzserver-1] [INFO] [1716719423.497694850] [gazebo_ros2_control]: Loading gazebo_ros2_control plugin
[gzserver-1] [INFO] [1716719423.506087554] [gazebo_ros2_control]: Starting gazebo_ros2_control plugin in namespace: /
[gzserver-1] [INFO] [1716719423.507041445] [gazebo_ros2_control]: Starting gazebo_ros2_control plugin in ros 2 node: gazebo_ros2_control

[gzserver-1] [INFO] [1716719423.512522059] [gazebo_ros2_control]: connected to service!! robot_state_publisher
[gzserver-1] [INFO] [1716719423.514977965] [gazebo_ros2_control]: Received urdf from param server, parsing...
[gzserver-1] [INFO] [1716719423.516029337] [gazebo_ros2_control]: Loading parameter files /home/trantor/project/gazebo_ws/install/gazebo_ros2_control_demos/share/gazebo_ros2_control_demos/config/diff_drive_controller.yaml
[gzserver-1] [INFO] [1716719423.539644869] [gazebo_ros2_control]: Loading joint: left_wheel_joint
[gzserver-1] [INFO] [1716719423.540748062] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1716719423.540885359] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1716719423.541329234] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1716719423.541522809] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1716719423.541668362] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1716719423.543291915] [gazebo_ros2_control]: Loading joint: right_wheel_joint
[gzserver-1] [INFO] [1716719423.543449626] [gazebo_ros2_control]: 	State:
[gzserver-1] [INFO] [1716719423.543480451] [gazebo_ros2_control]: 		 position
[gzserver-1] [INFO] [1716719423.543526921] [gazebo_ros2_control]: 		 velocity
[gzserver-1] [INFO] [1716719423.543547684] [gazebo_ros2_control]: 	Command:
[gzserver-1] [INFO] [1716719423.543588297] [gazebo_ros2_control]: 		 velocity

[gzserver-1] [INFO] [1716719423.544598676] [resource_manager]: Initialize hardware 'GazeboSystem'
[gzserver-1] [INFO] [1716719423.546728682] [resource_manager]: Successful initialization of hardware 'GazeboSystem'
[gzserver-1] [INFO] [1716719423.547218203] [resource_manager]: 'configure' hardware 'GazeboSystem'
[gzserver-1] [INFO] [1716719423.547237541] [resource_manager]: Successful 'configure' of hardware 'GazeboSystem'
[gzserver-1] [INFO] [1716719423.547244087] [resource_manager]: 'activate' hardware 'GazeboSystem'
[gzserver-1] [INFO] [1716719423.547248674] [resource_manager]: Successful 'activate' of hardware 'GazeboSystem'
[gzserver-1] [INFO] [1716719423.547942468] [gazebo_ros2_control]: Loading controller_manager
[gzserver-1] [WARN] [1716719423.565643044] [gazebo_ros2_control]:  Desired controller update period (0.01 s) is slower than the gazebo simulation period (0.001 s).
[gzserver-1] [INFO] [1716719423.566317969] [gazebo_ros2_control]: Loaded gazebo_ros2_control.

[INFO] [spawn_entity.py-4]: process has finished cleanly [pid 32751]
[INFO] [ros2-5]: process started with pid [32879]

[gzclient-2] context mismatch in svga_surface_destroy
[gzclient-2] context mismatch in svga_surface_destroy

[gzserver-1] [INFO] [1716719424.957344379] [controller_manager]: Loading controller 'joint_state_broadcaster'
[gzserver-1] [INFO] [1716719424.979672228] [controller_manager]: Configuring controller 'joint_state_broadcaster'
[gzserver-1] [INFO] [1716719424.980039786] [joint_state_broadcaster]: 'joints' or 'interfaces' parameter is empty. All available state interfaces will be published
[ros2-5] Successfully loaded controller joint_state_broadcaster into state active

[INFO] [ros2-5]: process has finished cleanly [pid 32879]
[INFO] [ros2-6]: process started with pid [32939]

[gzserver-1] [INFO] [1716719426.151156845] [controller_manager]: Loading controller 'diff_drive_base_controller'
[gzserver-1] [INFO] [1716719426.181163296] [controller_manager]: Configuring controller 'diff_drive_base_controller'
[ros2-6] Successfully loaded controller diff_drive_base_controller into state active
[INFO] [ros2-6]: process has finished cleanly [pid 32939]
```
