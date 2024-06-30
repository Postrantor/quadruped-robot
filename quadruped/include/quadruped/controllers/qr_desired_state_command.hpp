/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_DESIRED_STATE_COMMAND_H
#define QR_DESIRED_STATE_COMMAND_H

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

#include "config/qr_config.h"
#include "config/qr_enum_types.h"
#include "robots/qr_robot.h"
#include "utils/qr_cpptypes.h"

namespace Quadruped {
union JOYData {
  int value[3];
  char data_buffer[12];
};

class qrDesiredStateCommand {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief 构造函数，创建 qrDesiredStateCommand 类的实例
   * @param nhIn: ROS 节点句柄，用于订阅 joy 节点
   * @param robotIn: 机器人指针，主要用于获取机器人高度并设置期望高度
   */
  qrDesiredStateCommand(const rclcpp::Node::SharedPtr &nhIn, qrRobot *robotIn);

  ~qrDesiredStateCommand() = default;

  /**
   * @brief 根据 joy 输入更新期望状态
   */
  void Update();

  /**
   * @brief 打印来自 joy 的命令
   */
  void PrintRawInfo();

  /**
   * @brief 打印由 joy 命令生成的期望状态
   */
  void PrintStateCommandInfo();

  /**
   * @brief joy 命令接收到时调用的回调函数
   * @param joy_msg: 来自 ROS joy_node 的 joy 命令消息
   */
  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr &joy_msg);

  int RecvSocket();

  /**
   * @brief joyCtrlState 成员的获取方法
   */
  inline RC_MODE getJoyCtrlState() const { return joyCtrlState; }

  /**
   * @brief joyCtrlStateChangeRequest 成员的获取方法
   */
  inline bool getJoyCtrlStateChangeRequest() const { return joyCtrlStateChangeRequest; }

  /**
   * @brief joyCtrlStateChangeRequest 成员的设置方法
   * @param request: joyCtrlStateChangeRequest 的新值
   */
  inline void setJoyCtrlStateChangeRequest(bool request) { joyCtrlStateChangeRequest = request; }

  /**
   * @brief 表示在机体坐标系中表示的期望线性速度
   */
  Vec3<float> vDesInBodyFrame;

  /**
   * @brief 表示在机体坐标系中表示的期望角速度
   */
  Vec3<float> wDesInBodyFrame;

  /**
   * @brief 存储期望位置、滚转角、线性速度和滚转角速度
   */
  Vec12<float> stateDes;

  /**
   * @brief 存储当前位置、滚转角、线性速度和滚转角速度
   */
  Vec12<float> stateCur;

  /**
   * @brief 存储上一个期望位置、滚转角、线性速度和滚转角速度
   */
  Vec12<float> preStateDes;

  /**
   * @brief 存储线性和角加速度，以及 12 个关节加速度
   */
  Vec18<float> ddqDes;

  /**
   * @brief 期望关节角度，当前只在步行运动中使用，可能在将来被废弃
   */
  Eigen::Matrix<float, 3, 4> legJointq;

  /**
   * @brief 期望关节角速度，当前未使用，可能在将来被废弃
   */
  Eigen::Matrix<float, 3, 4> legJointdq;

  /**
   * @brief 表示在世界坐标系中表示的期望足端目标位置
   */
  Eigen::Matrix<float, 3, 4> footTargetPositionsInWorldFrame;

  // char data_buffer[4*3]; // 1024
  int socket_value[3] = {0, 0, 0};
  JOYData joyData;

private:
  /**
   * @brief 控制循环的时间步长
   * @todo 将此值同步到机器人类中的时间步长
   * @see Quadruped::Robot
   */
  float dt = 0.002f;

  /**
   * @brief 订阅 "/joy" 的 ROS 节点句柄
   * @todo 将使用驱动程序获取 joy 的状态，而不是 ROS 话题
   */
  const rclcpp::Node::SharedPtr &nh;

  /**
   * @brief ROS joy_node 的订阅者
   */
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepadCommandSub;

  /**
   * @brief ROS joy_node 的话题名称
   */
  std::string topicName = "/joy";

  /**
   * @brief 当前 joy 状态
   */
  RC_MODE joyCtrlState;

  /**
   * @brief 上一个 joy 状态
   */
  RC_MODE prevJoyCtrlState;

  /**
   * @brief 一个变量，指示四足机器人是否正在移动
   */
  int movementMode;

  /**
   * @brief 一个变量，指示四足机器人是否需要改变状态。如果该变量为真，它将用于更新四足机器人的有限状态机。
   */
  bool joyCtrlStateChangeRequest;

  /**
   * @brief 一个变量，指示是否使用 ROS 消息控制或 joy 控制
   * @attention 目前，我们在四足机器人库中只使用 joy 控制。如果您想使用其他控制方法，请自行实现。
   */
  bool joyCtrlOnRequest;

  /**
   * @brief 与 joyCtrlOnRequest 相反
   */
  bool rosCmdRequest;

  /**
   * @brief 一个变量，指示 joy 控制是否停止。当这个布尔值为真时，joy 命令将设置为零，四足机器人将停止运动。
   */
  bool joyCmdExit;

  /**
   * @brief 一个变量，指示机体是向上还是向下。如果 bodyUp 等于 -1，四足机器人正在蹲下。如果 bodyUp 等于 0，四足机器人在
   * bodyHeight 处停止。如果 bodyUp 等于 1，四足机器人正在站起来。
   */
  int bodyUp;

  /**
   * @brief 一个变量，指示四足机器人是否需要改变步态。当前这个变量没有实际用途。将来可能会移除这个成员。
   * @todo 将来会移除这个成员
   */
  bool gaitSwitch = false;

  bool isSim;

  /**
   * @brief 期望的机体高度，现在是一个常数
   */
  float joycmdBodyHeight;

  /**
   * @brief 来自 joy 控制的 Z 轴上的线性速度
   */
  float joyCmdVz;

  /**
   * @brief 来自 joy 控制的 X 轴上的线性速度
   */
  float joyCmdVx;

  /**
   * @brief 来自 joy 控制的 Y 轴上的线性速度
   */
  float joyCmdVy;

  /**
   * @brief 来自 joy 控制的偏航角速度
   */
  float joyCmdYawRate;

  /**
   * @brief 来自 joy 控制的滚转角速度
   */
  float joyCmdRollRate;

  /**
   * @brief 来自 joy 控制的俯仰角速度
   */
  float joyCmdPitchRate;

  /**
   * @brief 此因子用作当前速度和 joy 速度之间的线性因子
   */
  const float filterFactor = 0.02f;

  /**
   * @brief 从 joy 命令中过滤后的线性速度。
   */
  Vec3<float> filteredVel;

  /**
   * @brief 从 joy 命令中过滤后的角速度。
   * @attention 这实际上是滚转、俯仰和偏航的变化率。
   */
  Vec3<float> filteredOmega;

  /**
   * @brief 选择每隔多少次迭代打印信息。
   */
  int printNum = 5;

  /**
   * @brief 跟踪自上次打印信息以来的迭代次数。
   */
  int printIter = 0;

  /**
   * @brief 期望状态的最大滚转值。
   */
  const float MAX_ROLL = 0.4f;

  /**
   * @brief 期望状态的最小滚转值。
   */
  const float MIN_ROLL = -0.4f;

  /**
   * @brief 期望状态的最大俯仰值。
   */
  const float MAX_PITCH = 0.5f;

  /**
   * @brief 期望状态的最小俯仰值。
   */
  const float MIN_PITCH = -0.5f;

  /**
   * @brief 期望状态的 X 轴上的最大线性速度。
   */
  const float MAX_VELX = 0.2f;

  /**
   * @brief 期望状态的 X 轴上的最小线性速度。
   */
  const float MIN_VELX = -0.15f;

  /**
   * @brief 期望状态的 Y 轴上的最大线性速度。
   */
  const float MAX_VELY = 0.1f;

  /**
   * @brief 期望状态的 Y 轴上的最小线性速度。
   */
  const float MIN_VELY = -0.1f;

  /**
   * @brief 期望状态的最大偏航速度。
   */
  const float MAX_YAWRATE = 0.2f;

  /**
   * @brief 期望状态的最小偏航速度。
   */
  const float MIN_YAWRATE = -0.2f;

  /**
   * @brief 期望状态的最大滚转速度。
   */
  const float MAX_ROLLRATE = 0.2f;

  /**
   * @brief 期望状态的最小滚转速度。
   */
  const float MIN_ROLLRATE = -0.2f;

  /**
   * @brief 期望状态的最大俯仰速度。
   */
  const float MAX_PITCHRATE = 0.2f;

  /**
   * @brief 期望状态的最小俯仰速度。
   */
  const float MIN_PITCHRATE = -0.2f;

  /**
   * @brief 期望状态的最大机体高度。
   * 此成员目前未使用。
   */
  const float BODY_HEIGHT_MAX = 0.5f;

  /**
   * @brief 期望状态的最小机体高度。
   * 此成员目前未使用。
   */
  const float BODY_HEIGHT_MIN = 0.15f;
};

}  // Namespace Quadruped

#endif  // QR_DESIRED_STATE_COMMAND_H
