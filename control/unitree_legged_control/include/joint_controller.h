/**
 * @brief
 * @copyright
 */

#ifndef _UNITREE_ROS_JOINT_CONTROLLER_H_
#define _UNITREE_ROS_JOINT_CONTROLLER_H_

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include "ros/node_handle.h"
#include "urdf/model.h"
#include "control_toolbox/pid.h"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_buffer.h"

#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/WrenchStamped.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"

#include "unitree_joint_control_tool.h"

namespace unitree_legged_control {

class UnitreeJointController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
private:
  hardware_interface::JointHandle joint;
  ros::Subscriber sub_cmd, sub_ft;
  // ros::Publisher pub_state;
  control_toolbox::Pid pid_controller_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<unitree_legged_msgs::MotorState> >
      controller_state_publisher_;

public:
  UnitreeJointController();
  ~UnitreeJointController();

  // bool start_up;
  std::string name_space;
  std::string joint_name;
  float sensor_torque;
  bool isHip, isThigh, isCalf, rqtTune;
  urdf::JointConstSharedPtr joint_urdf;

  // pub?
  realtime_tools::RealtimeBuffer<unitree_legged_msgs::MotorCmd> command;
  unitree_legged_msgs::MotorCmd lastCmd;
  unitree_legged_msgs::MotorState lastState;

  ServoCmd servoCmd;

  virtual bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
  virtual void starting(const ros::Time &time);
  virtual void update(const ros::Time &time, const ros::Duration &period);
  virtual void stopping();

  void setTorqueCB(const geometry_msgs::WrenchStampedConstPtr &msg);
  void setCommandCB(const unitree_legged_msgs::MotorCmdConstPtr &msg);

  void positionLimits(double &position);
  void velocityLimits(double &velocity);
  void effortLimits(double &effort);

  void setGains(
      const double &p,
      const double &i,
      const double &d,
      const double &i_max,
      const double &i_min,
      const bool &antiwindup = false);
  void getGains(
      double &p,      //
      double &i,      //
      double &d,      //
      double &i_max,  //
      double &i_min,  //
      bool &antiwindup);
  void getGains(
      double &p,      //
      double &i,      //
      double &d,      //
      double &i_max,  //
      double &i_min);
};
}  // namespace unitree_legged_control

#endif
