/**
 * @brief
 * @date 2024-02-25
 * @copyright Copyright (c) 2017, Alexander W. Winkler. All rights reserved.
 */

#include <ros/ros.h>

#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <xpp_states/convert.h>
#include <xpp_vis/rviz_robot_builder.h>


static ros::Publisher rviz_marker_pub;
static xpp::RvizRobotBuilder robot_builder;

static void StateCallback (const xpp_msgs::RobotStateCartesian& state_msg)
{
  auto rviz_marker_msg = robot_builder.BuildRobotState(state_msg);
  rviz_marker_pub.publish(rviz_marker_msg);
}

static void TerrainInfoCallback (const xpp_msgs::TerrainInfo& terrain_msg)
{
  robot_builder.SetTerrainParameters(terrain_msg);
}

static void ParamsCallback (const xpp_msgs::RobotParameters& params_msg)
{
  robot_builder.SetRobotParameters(params_msg);
}

int main(int argc, char *argv[])
{
  using namespace ros;

  init(argc, argv, "rviz_marker_visualizer");

  NodeHandle n;

  Subscriber parameters_sub;
  parameters_sub = n.subscribe(xpp_msgs::robot_parameters, 1, ParamsCallback);

  Subscriber state_sub_curr, state_sub_des, terrain_info_sub;
  state_sub_des     = n.subscribe(xpp_msgs::robot_state_desired, 1, StateCallback);
  terrain_info_sub  = n.subscribe(xpp_msgs::terrain_info, 1,  TerrainInfoCallback);

  rviz_marker_pub = n.advertise<visualization_msgs::MarkerArray>("xpp/rviz_markers", 1);

  spin();

  return 1;
}
