### ros2 log

```log
> ros2 run robot_sim example_gazebo
[INFO] [1722778487.474902934] [robot_sim]: starting robot_sim node.
[INFO] [1722778487.475135172] [robot_sim]: initialize and reset robot.
[INFO] [1722778487.481695923] [robot_sim]: joint state set successfully
[INFO] [1722778487.482381485] [robot_sim]: find package path in:
	- `/home/trantor/project/quadruped_ws/install/quadruped/share/quadruped`
[INFO] [1722778487.482439141] [robot_sim]: config file path:
	- `/home/trantor/project/quadruped_ws/install/quadruped/share/quadruped/../../config/a1_sim/a1_sim.yaml`
[INFO] [1722778487.483638772] [robot_sim]:
	inertias: [0.000469, -0.000009, -0.000000, -0.000009, 0.000807, -0.000000, -0.000000, -0.000000, 0.000553]
[INFO] [1722778487.797261239] [robot_sim]:
	yaw offset: 0
[INFO] [1722778487.797361974] [robot_sim]: sim robot object init complete.
[INFO] [1722778487.799028688] [robot_sim]: entry control loop.
[INFO] [1722778487.799531407] [robot_sim]:
	base orientation: [1, 0, 0, 0]
[INFO] [1722778487.799900089] [robot_sim]: get package config dir:
	- `/home/trantor/project/quadruped_ws/install/quadruped/share/quadruped/../../`
init UserParameters finish
[Desired State Command] init finish...
[INFO] [1722778487.800738539] [robot_sim]: [Runner] name: a1_sim
[INFO] [1722778487.800782015] [robot_sim]: load yaml: /home/trantor/project/quadruped_ws/install/quadruped/share/quadruped/../../config/a1_sim/main.yaml
---------------------Standing Up---------------------
---------------------Stand Up Finished---------------------
OpenLoopGaitGenerator Set gait: advanced_trot
init gaitGenerator finish
init groundEsitmator finish
init detection finish
estimatedPosition =    0    0 0.27
init robotEstimator finish
init state estimator container!
[FSM_State] Initialized FSM state: PASSIVE
[FSM_State] Initialized FSM state: STAND_UP
[FSM_State] Initialized FSM state: LOCOMOTION
init comAdjuster finish
poseDest =    0    0 0.27    0    0    0
init posePlanner finish
init footholdPlanner finish
/home/trantor/project/quadruped_ws/install/quadruped/share/quadruped/../..//config/a1_sim/swing_leg_controller.yaml
init swingLegController finish
[Convex MPC]:
	dt: 0.002,
	iterations: 30,
	dtMPC: 0.06,
	horizonLen: 5
[MPC] Reset: iterationCounter = 0
[MPC] Init: success!
init stanceLegController finish
init locomotionController finish
LocomotionController Init Finished
BuildDynamicModel Init BEFORE
[INFO] [1722778491.815314631] [robot_sim]: hip inertia:
   0.005529   4.825e-06   0.0003439
4.82499e-06   0.0051393 2.23999e-05
  0.0003439    2.24e-05   0.0013678
--- --- ---
    0.005529   -4.825e-06    0.0003439
-4.82499e-06    0.0051393 -2.23999e-05
   0.0003439    -2.24e-05    0.0013678
BuildDynamicModel Init Finished
WbcLocomotionCtrl Init Finished
in OnEnter
ENTER the standup fsm!!!
FSM Init FinishedStateEstimatorContainer Reset
OpenLoopGaitGenerator Set gait: advanced_trot
[MPC] Reset: iterationCounter = 0
[INFO] [1722778491.815800869] [robot_sim]: LocomotionController Init Finished
[INFO] [1722778491.815852154] [robot_sim]: ROS Modules Init Finished
[INFO] [1722778491.815875003] [robot_sim]: TimeSinceReset: 4.01852
[INFO] [1722778491.818059321] [robot_sim]: loop rate 1000
[INFO] [1722778491.818127929] [robot_sim]: start control loop... ...
---------------------Standing Up---------------------
---------------------Stand Up Finished---------------------
[ERROR] [1722778493.811345418] [robot_sim]: the dog is going down, main function exit...
[INFO] [1722778493.811458122] [robot_sim]:
	base pos:
	-0.0406364
-0.0291286
       0.4
	base rpy:
	0
0
0

```

### ros1 log

```log
> rosrun examples example_a1_sim
[ INFO] [1722766527.739072475]: BRILLIANT!!!
[ INFO] [1722766527.753007243]: Reset the Robot pose
---------ROS node init finished---------
0.000469246
-9.409e-06
-3.42e-07
-9.409e-06
0.00080749
-4.66e-07
-3.42e-07
-4.66e-07
0.000552929
yawOffset: 0
-------A1Sim init Complete-------
robot created........
BaseOrientation:
1 0 0 0
init UserParameters finish
[Desired State Command] init finish...
[Runner] name: a1_sim
/home/trantor/catkin_ws/src/quadruped-robot/model-control/src/quadruped/config/a1_sim/main.yaml
---------------------Standing Up---------------------
---------------------Stand Up Finished---------------------
OpenLoopGaitGenerator Set gait: advanced_trot
init gaitGenerator finish
init groundEsitmator finish
init detection ---
estimatedPosition =    0    0 0.27
init robotEstimator finish
init state estimator container!

[FSM_State] Initialized FSM state: PASSIVE
[FSM_State] Initialized FSM state: STAND_UP
[FSM_State] Initialized FSM state: LOCOMOTION

init comAdjuster finish
poseDest =    0    0 0.27    0    0    0
init posePlanner finish
init footholdPlanner finish
init swingLegController finish

[Convex MPC] dt: 0.002 iterations: 30, dtMPC: 0.060, horizonLen: 5
[MPC] Reset: iterationCounter = 0
[MPC] Init: success!
init stanceLegController finish
init locomotionController finish
LocomotionController Init Finished
BuildDynamicModel Init BEFORE
hipInertia -----
   0.005529   4.825e-06   0.0003439
4.82499e-06   0.0051393 2.23999e-05
  0.0003439    2.24e-05   0.0013678
    0.005529   -4.825e-06    0.0003439
-4.82499e-06    0.0051393 -2.23999e-05
   0.0003439    -2.24e-05    0.0013678
----- hipInertia
BuildDynamicModel Init Finished
WbcLocomotionCtrl Init Finished
in OnEnter
ENTER the standup fsm!!!
FSM Init FinishedStateEstimatorContainer Reset
OpenLoopGaitGenerator Set gait: advanced_trot
[MPC] Reset: iterationCounter = 0
[ INFO] [1722766532.749167405, 104.351000000]: loop rate 1000.000000

[ INFO] [1722766532.749692739, 104.351000000]: LocomotionController Init Finished
[ INFO] [1722766532.757847063, 104.351000000]: Controller2GazeboMsg init success...
[ INFO] [1722766532.758072777, 104.351000000]: ROS Modules Init Finished
[ INFO] [1722766532.761585803, 104.351000000]: TimeSinceReset: 104.350998
[ INFO] [1722766534.827090604, 104.893000000]: start control loop....
---------------------Standing Up---------------------
---------------------Stand Up Finished---------------------

joyCtrlState before = 5, movementMode = 2
[RunFSM]: RC_MODE = 1
STAND_UP EXIT!
[FSM] On Enter State: 1
Time Step: 0.001
OpenLoopGaitGenerator Set gait: trot
StateEstimatorContainer Reset
[FSM LOCOMOTION] On Enter End
FSM_State_Locomotion: reset iter for GAIT_TRANSITION!!!
[ INFO] [1722766540.497478796, 105.451000000]: You have pressed the stop button!!!!

joyCtrlState before = 1, movementMode = 1
[RunFSM]: RC_MODE = 4
[ INFO] [1722766540.729782065, 105.459000000]: transitionData.done
Locomotion Exit
[FSM] On Enter State: 4
Time Step: 0.001
OpenLoopGaitGenerator Set gait: stand
StateEstimatorContainer Reset
[FSM LOCOMOTION] On Enter End
[ INFO] [1722766541.063946909, 105.475000000]: You have change the gait !!!

joyCtrlState before = 4, movementMode = 2
[RunFSM]: RC_MODE = 1
FSM_State_Locomotion: reset iter for GAIT_TRANSITION!!!
[ INFO] [1722766555.741977646, 107.149000000]: You have pressed the stop button!!!!

joyCtrlState before = 1, movementMode = 1
[RunFSM]: RC_MODE = 4
[ INFO] [1722766555.833737192, 107.161000000]: transitionData.done
Locomotion Exit
[FSM] On Enter State: 4
Time Step: 0.001
OpenLoopGaitGenerator Set gait: stand
```
