/**
 * @brief []
 * @author unitree
 * @date 2024-06-23 16:06:13
 * @copyright Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
    Use of this source code is governed by the MPL-2.0 license, see LICENSE.
 */

#include "joint_controller.h"
#include <pluginlib/class_list_macros.h>

// #define rqtTune // use rqt or not

namespace unitree_legged_control {

UnitreeJointController::UnitreeJointController() {
  memset(&lastCmd, 0, sizeof(unitree_legged_msgs::MotorCmd));
  memset(&lastState, 0, sizeof(unitree_legged_msgs::MotorState));
  memset(&servoCmd, 0, sizeof(ServoCmd));
}

UnitreeJointController::~UnitreeJointController() {
  sub_ft.shutdown();
  sub_cmd.shutdown();
}

/**
 * @brief 同`setCommandCB`?
 * @details 但是这里赋值的变量`sensor_torque`并没有参与控制循环的计算，应该暂时没有使用上？
 * @param msg
 */
void UnitreeJointController::setTorqueCB(const geometry_msgs::WrenchStampedConstPtr &msg) {
  if (isHip)
    sensor_torque = msg->wrench.torque.x;
  else
    sensor_torque = msg->wrench.torque.y;
}

/**
 * @brief 从指令话题("command")获取期望指令，用与控制循环内的计算
 * @details 通过sub的回调接收cmd指令，赋值给lastCmd，再借由command.writeFromNonRT()的API写入内部变量？
 *        之后通过update()->command.readFromRT()的API读取指令，用与在当前循环中计算控制命令值
 *        这两个API的使用可以保证实时？
 * @param msg
 */
void UnitreeJointController::setCommandCB(const unitree_legged_msgs::MotorCmdConstPtr &msg) {
  lastCmd.mode = msg->mode;
  lastCmd.q = msg->q;
  lastCmd.Kp = msg->Kp;
  lastCmd.dq = msg->dq;
  lastCmd.Kd = msg->Kd;
  lastCmd.tau = msg->tau;
  // writeFromNonRT 可以在 RT 中使用，前提是你能保证
  // - 没有非 RT 线程调用同一个函数（我们没有订阅 ros 回调）
  // - 只有一个 RT 线程
  command.writeFromNonRT(lastCmd);
}

/**
 * @brief 设置控制算法参数
 * @details 这几个和控制算法相关的都没用上，update()中没有使用
 * @param p
 * @param i
 * @param d
 * @param i_max
 * @param i_min
 * @param antiwindup
 */
void UnitreeJointController::setGains(
    const double &p,
    const double &i,
    const double &d,
    const double &i_max,
    const double &i_min,
    const bool &antiwindup) {
  pid_controller_.setGains(p, i, d, i_max, i_min, antiwindup);
}

/**
 * @brief 获取控制算法参数
 * @param p
 * @param i
 * @param d
 * @param i_max
 * @param i_min
 * @param antiwindup
 */
void UnitreeJointController::getGains(  //
    double &p,                          //
    double &i,                          //
    double &d,                          //
    double &i_max,                      //
    double &i_min,                      //
    bool &antiwindup) {
  pid_controller_.getGains(p, i, d, i_max, i_min, antiwindup);
}

void UnitreeJointController::getGains(  //
    double &p,                          //
    double &i,                          //
    double &d,                          //
    double &i_max,                      //
    double &i_min) {
  bool dummy;
  pid_controller_.getGains(p, i, d, i_max, i_min, dummy);
}

/**
 * @brief UnitreeJointController 初始化
 *   1. 从urdf文件中获取joint相关信息，通过验证后，获取对应的handles
 *      这个handles是与hardware结合的点
 *   2. Start command subscriber
 *      2.1 从gazebo中获取执行机构的状态(力)
 *      2.2 从上位机获取期望指令
 *   3. 发布机器人状态数据(MotorState)
 * @param robot
 * @param n
 * @return true
 * @return false
 */
// Controller initialization in non-realtime
bool UnitreeJointController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n) {
  isHip = false;
  isThigh = false;
  isCalf = false;

  sensor_torque = 0;
  name_space = n.getNamespace();
  if (!n.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }

  // Load PID Controller using gains set on parameter server
  // if (!pid_controller_.init(ros::NodeHandle(n, "pid"))) return false;

  urdf::Model urdf;  // Get URDF info about joint
  if (!urdf.initParamWithNodeHandle("robot_description", n)) {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf = urdf.getJoint(joint_name);
  if (!joint_urdf) {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }
  if (joint_name == "FR_hip_joint" || joint_name == "FL_hip_joint" || joint_name == "RR_hip_joint" ||
      joint_name == "RL_hip_joint") {
    isHip = true;
  }
  if (joint_name == "FR_calf_joint" || joint_name == "FL_calf_joint" || joint_name == "RR_calf_joint" ||
      joint_name == "RL_calf_joint") {
    isCalf = true;
  }
  /// 1. 从urdf文件中获取joint相关信息，通过验证后，获取对应的handles
  /// 这个handles是与hardware结合的点
  joint = robot->getHandle(joint_name);

  /// 2. Start command subscriber
  /// 2.1 从gazebo中获取执行机构的状态(力)
  /// 2.2 从上位机获取期望指令
  sub_ft = n.subscribe(name_space + "/" + "joint_wrench", 1, &UnitreeJointController::setTorqueCB, this);
  sub_cmd = n.subscribe("command", 20, &UnitreeJointController::setCommandCB, this);

  /// 3. Start realtime state publisher
  /// 发布机器人状态数据(MotorState)
  // pub_state = n.advertise<unitree_legged_msgs::MotorState>(name_space + "/state", 20);
  controller_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<unitree_legged_msgs::MotorState>(n, name_space + "/state", 1));

  return true;
}

/**
 * @brief 初始化控制器
 * @details Controller startup in realtime
 * @param time
 */
void UnitreeJointController::starting(const ros::Time &time) {
  double init_pos = joint.getPosition();
  // lastCmd.Kp = 0;
  // lastCmd.Kd = 0;
  lastCmd.q = init_pos;
  lastState.q = init_pos;
  lastCmd.dq = 0;
  lastState.dq = 0;
  lastCmd.tau = 0;
  lastState.tauEst = 0;
  command.initRT(lastCmd);

  pid_controller_.reset();
}

/**
 * @brief 更新控制器
 * @details Controller update loop in realtime
 *   1. 获取期望指令
 *   2. 依据电机的状态\限位等信息，依据cmd指令计算出许用的指令参数servoCmd
 *   3*. 依据电机状态信息/许用值估算(微分)出电机受力状态信息
 *   4. 将计算出的 控制力矩数值 作为控制指令发送给电机
 *   5. 保留当前状态数据作为上一次计算数据，用于下一次控制循环计算
 *   6. publish state
 * @param time
 * @param period
 */
void UnitreeJointController::update(const ros::Time &time, const ros::Duration &period) {
  /// 1. 获取期望指令
  lastCmd = *(command.readFromRT());

  /// 2. 依据电机的状态\限位等信息，依据cmd指令计算出许用的指令参数servoCmd
  // set command data
  if (lastCmd.mode == PMSM) {
    servoCmd.pos = lastCmd.q;
    positionLimits(servoCmd.pos);
    servoCmd.posStiffness = lastCmd.Kp;
    if (fabs(lastCmd.q - PosStopF) < 0.00001) {
      servoCmd.posStiffness = 0;
    }
    servoCmd.vel = lastCmd.dq;
    velocityLimits(servoCmd.vel);
    servoCmd.velStiffness = lastCmd.Kd;
    if (fabs(lastCmd.dq - VelStopF) < 0.00001) {
      servoCmd.velStiffness = 0;
    }
    servoCmd.torque = lastCmd.tau;
    effortLimits(servoCmd.torque);
  }
  if (lastCmd.mode == BRAKE) {
    servoCmd.posStiffness = 0;
    servoCmd.vel = 0;
    servoCmd.velStiffness = 0;
    servoCmd.torque = 0;
    effortLimits(servoCmd.torque);
  }

  /// *3. 依据电机状态信息/许用值估算(微分)出电机受力状态信息
  double currentPos;
  double currentVel;
  double calcTorque;
  currentPos = joint.getPosition();
  currentVel = computeVel(currentPos, (double)lastState.q, (double)lastState.dq, period.toSec());
  calcTorque = computeTorque(currentPos, currentVel, servoCmd);
  effortLimits(calcTorque);

  /// 4. 将计算出的 控制力矩数值 作为控制指令发送给电机
  joint.setCommand(calcTorque);

  /// 5. 保留当前状态数据作为上一次计算数据，用于下一次控制循环计算
  lastState.q = currentPos;
  lastState.dq = currentVel;
  lastState.tauEst = joint.getEffort();
  // lastState.tauEst = calcTorque;
  // lastState.tauEst = sensor_torque;

  /// 6. publish state
  // pub_state.publish(lastState);
  if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
    controller_state_publisher_->msg_.q = lastState.q;
    controller_state_publisher_->msg_.dq = lastState.dq;
    controller_state_publisher_->msg_.tauEst = lastState.tauEst;
    controller_state_publisher_->unlockAndPublish();
  }

  // printf("sensor torque%f\n", sensor_torque);
  // if(joint_name == "wrist1_joint") printf("wrist1 setp:%f  getp:%f t:%f\n", servoCmd.pos, currentPos, calcTorque);
}

/**
 * @brief 停止控制器
 * @details Controller stopping in realtime
 */
void UnitreeJointController::stopping() {}

/**
 * @brief 限位
 * @details 通过joint_urdf获取urdf中设置的限位参数，使用clamp()函数计算获得限位值
 * @param position
 */
void UnitreeJointController::positionLimits(double &position) {
  if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
    clamp(position, joint_urdf->limits->lower, joint_urdf->limits->upper);
}

/**
 * @brief 限位
 * @param velocity
 */
void UnitreeJointController::velocityLimits(double &velocity) {
  if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
    clamp(velocity, -joint_urdf->limits->velocity, joint_urdf->limits->velocity);
}

/**
 * @brief 限位
 * @param effort
 */
void UnitreeJointController::effortLimits(double &effort) {
  if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
    clamp(effort, -joint_urdf->limits->effort, joint_urdf->limits->effort);
}

}  // namespace unitree_legged_control

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(unitree_legged_control::UnitreeJointController, controller_interface::ControllerBase);
