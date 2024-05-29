Installation
************

Installing ROS
===========

You need install ROS (Robot Operating System) first. We tested the codes under Ubuntu Linux and ROS 1 Melodic Morenia distribution. Other newer ROS distributions are supposed to be supported. Please visit http://www.wiki.ros.org for ROS installation.

Cloning the source code.
======================

.. code-block:: console

    cd ${your_workspace}
    mkdir src
    cd ${your_workspace}/src
    catkin_init_workspace
    git clone https://github.com/TopHillRobotics/quadruped-robot/

Installing the following third party dependencies.
===============================================

- Eigen3
- yaml-cpp
- lcm
- Quadprogpp
- ros

.. code-block:: console

    sudo apt install libyaml-cpp-dev
    sudo apt install libeigen3-dev
    sudo apt install liblcm-dev
    sudo apt install libglm-dev

Building the codes.
==================

Make sure your cmake version is 3.15.7 or higher

.. code-block:: console

    cd ${your_workspace}
    catkin_make

Run ``source devel/setup.bash`` before launching the project.