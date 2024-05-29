.. _helloworld_real_reference-label:

Hello World in Real Environments
*********************

The Hello World demo (``demo/demo_helloworld``) is also a good exaple to understand the deployment of our code on a real quadruped robot. Here, we also use a Unitree A1 quadruped robot to explain the process.

Initializing the ROS node
=======================

Since our code is developped in ROS, we need initializing the ROS node first. Using ROS components, our program can communicate to other programs.

.. code-block:: c++

    ros::init(argc, argv, "demo_helloworld");
    ros::NodeHandle nh;

Getting the YMAL path
============

Every demo has its own configuration and parameters. These parameters are stored in a ``.yaml`` file. We need tell where the ``.yaml`` is and later we can load parameters from the given ``.yaml`` file. In this example, a Unitree A1 robot is used and its configurations are store in ``demo_helloworld/real_config`` for a real robot. We set a robot name as A1, then we will get the directory ``demo/demo_helloworld``.

.. code-block:: c++

    std::string pathToPackage = ros::package::getPath("demo");
    std::string pathToNode =  pathToPackage + ros::this_node::getName();
    std::string robotName = "a1";

The first two lines are unnecessary in the Hello World demo. However, they may be useful in other demos.


Creating a robot
================

We set the parameter ``isSim`` to ``false`` on the ROS Parameter Server. Then we create a robot (Unitree A1) using the specified parameters stored in the  ``demo/demo_helloworld/real_config/main.yaml`` file.

.. code-block:: c++

    nh.setParam("isSim", false);
    qrRobot *quadruped = new qrRobotReal(robotName, LocomotionMode::VELOCITY_LOCOMOTION);


We also need initialize a few robot properties by receiving observation from the real robot.

.. code-block:: c++

    quadruped->ReceiveObservation();


Executing actions
===============

Now the initiliazation is finished. We are ready to execute standing actions. First, we let the robot perform the first action, standing up. It takes 3 seconds to stand up and keep 5 seconds before any other action. The parameter 0.001 is the specified time step (control frequency is 1000Hz). You may try different arguments to understand the action. 

.. code-block:: c++

    Action::StandUp(quadruped, 3.f, 5.f, 0.001f);

After standing up we let the quadruped robot keep standing for 20 seconds, and the control frequency is also 1000Hz.

.. code-block:: c++

    Action::StandUp(quadruped, 3.f, 5.f, 0.001f);

Finally the quadruped robot will sit down in 3 seconds with 1000Hz control frequency.

.. code-block:: c++

    Action::StandUp(quadruped, 3.f, 5.f, 0.001f);


Finishing and shutting down the ROS node
======================

After the demo is finished, we shut down the ROS nodes.

.. code-block:: c++

    ros::shutdown();


Launching the demo
=============

You can run a demo for a real quadruped robot either using your own external computer or a built-in miniPC (refer to A1 user manual). If you want to control the robot using your own computer, you need to connect your computer directly to the real quarduped robot with either an Ethernet cable or WiFi. This network connection allows a quarduped robot to communicate with your computer. Please refer to A1 user manual for how to build a connection. 

Make sure to keep the robot sufficiently charged and turn on the robot. 

To run the demo, in one terminal, source the setup.bash to set up the environment

.. code-block:: c++

    source ${your_workspace}/devel/setup.bash

Second, run roscore to start ROS since a real robot use LCM and ROS for communication. Please open a new terminal to run this command.

.. code-block:: c++

    roscore

Third, in a new terminal, launch a demo and run the quadruped controller node. Here, a demo helloworld lets the quadruped robot stand up.

.. code-block:: c++

    rosrun demo demo_helloworld real

Here, ``real`` indicates that the demo is running in a real quadruped robot. Given a robot YAML configuration file such as XACRO or URDF, a qrRobotReal object is constructed.
