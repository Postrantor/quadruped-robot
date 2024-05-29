Demos
*********************

Once you have conquered the HelloWorld example, either in simulation or in real environment, you should start looking at other demo examples. Each demo's name is formed of multiple words that are joined together to clearly hint what it does. These words are separating using "_" (underscore). Each demo's name begins with a word `demo_`, followed by two or three keywords. We encourage you to explore and make changes to these demos.  


demo_trot_velocity
==================

This demo shows the trotting gait of a quadruped robot on a flat ground. Through controlling the forward linear speed and rotating speed of the body or a quadruped robot, the body's trajectory and the supporting foot's position are computed using LocomotionMode::VELOCITY_LOCOMOTION for the next gait cycle. 

demo_trot_velocity_mpc
==================

This demo shows the trotting gait of a quadruped robot on a flat ground. Through controlling the forward linear speed and rotating speed of the body or a quadruped robot, the body's trajectory and the supporting foot's position are computed using LocomotionMode::VELOCITY_LOCOMOTION for the next gait cycle. Here, model predictive control (MPC) technique is used. 

demo_trot_position
==================

This demo shows the trotting gait of a quadruped robot on a flat ground. Through solving inverse kinematics, the body keeps balanced and the joint angles (by inverse kinematics) or torques (by inverse dyanmics) are computed using LocomotionMode::POSITION_LOCOMOTION.

demo_trot_keyboard
==================

This demo shows the trotting gait of a quadruped robot on a flat ground. You can use a keyboard to control the forward speed and rotating speed.

demo_trot_joystick
==================

This demo shows the trotting gait of a quadruped robot on a flat ground. You can use a game joystick to control the forward speed and rotating speed.

demo_external_force
==================

This demo shows how to apply external forces to a moving robot. Start this program when another demo has already started. 

demo_slam_publish_odom
==================

This demo shows how to pulish odemetry data. 

demo_slam_gmapping
==================

This demo shows how to build a map using SLAM and gmapping. 

demo_slam_cartographer
==================

This demo shows how to build a map using SLAM and cartographer. 

demo_terrain_slope
==================

This demo shows how to travel onto a slope. 

demo_terrain_stair
==================

This demo shows how to travel onto stairs. 

demo_terrain_rough
==================

This demo shows how to travel in a rough ground. 

demo_terrain_groove
==================

This demo shows how to walk inside a groove. 