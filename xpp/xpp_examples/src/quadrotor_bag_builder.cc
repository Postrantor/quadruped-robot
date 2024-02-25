/**
 * @brief
 * @date 2024-02-25
 * @copyright Copyright (c) 2017, Alexander W. Winkler. All rights reserved.
 */

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <xpp_msgs/RobotStateJoint.h>
#include <xpp_states/convert.h>
#include <xpp_states/state.h>


using namespace xpp;

int main(int argc, char *argv[])
{
  // creates a bag file wherever executable is run (usually ~/.ros/)
  rosbag::Bag bag;
  bag.open("quadrotor_traj.bag", rosbag::bagmode::Write);

  // visualize the state of a drone (only has a base)
  State3d base;

  // create a sequence of states for a total duration of T spaced 0.01s apart.
  double T = 2*M_PI;
  double dt = 0.01;
  double t = 1e-6;
  while (t < T)
  {
    // base and foot follow half a sine motion up and down
    base.lin.p_.z() = 0.7 - 0.5*sin(t);  //[m]
    base.lin.p_.x() = 1.0/T*t;           //[m]

    double roll = 30./180*M_PI*sin(t);
    base.ang.q = GetQuaternionFromEulerZYX(0.0, 0.0, roll);

    // save the state message with current time in the bag
    auto timestamp = ::ros::Time(t);

    xpp_msgs::RobotStateJoint msg;
    msg.base = Convert::ToRos(base);
    bag.write("xpp/joint_quadrotor_des", timestamp, msg);

    t += dt;
  }

  std::string bag_file = bag.getFileName(); // save location before closing
  bag.close();

  // plays back the states at desired speeds.
  // can now also use all the tools from rosbag to pause, speed-up, inspect,
  // debug the trajectory.
  // e.g. rosbag -play -r 0.2 /path/to/bag.bag
  int success = system(("rosbag play " + bag_file).c_str());

  return 0;
}

