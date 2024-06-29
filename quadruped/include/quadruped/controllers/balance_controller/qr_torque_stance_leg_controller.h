/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_TORQUE_STANCE_LEG_CONTROLLER_H
#define QR_TORQUE_STANCE_LEG_CONTROLLER_H

#include "controllers/qr_desired_state_command.hpp"
#include "estimators/qr_state_estimator_container.h"
#include "gait/qr_openloop_gait_generator.h"
#include "planner/qr_com_adjuster.h"
#include "planner/qr_foothold_planner.h"
#include "planner/qr_pose_planner.h"
#include "robots/qr_robot.h"
#include "utils/qr_se3.h"

namespace Quadruped {

class TorqueStanceLegController {
public:
  /**
   * @brief 构造函数 TorqueStanceLegController
   * @param robot: 机器人指针。
   * @param gaitGenerator: 步态生成器指针。
   * @param stateEstimators: 状态估计器容器指针。
   * @param comAdjuster: COM 调整器指针。
   * @param posePlanner: 姿态规划器指针。
   * @param footholdPlanner: 脚部规划器指针。
   * @param userParameters: 用户参数引用。
   * @param configFilepath: 配置文件路径字符串。
   */
  TorqueStanceLegController(
      qrRobot *robot,
      qrGaitGenerator *gaitGenerator,
      qrStateEstimatorContainer *stateEstimators,
      qrComAdjuster *comAdjuster,
      qrPosePlanner *posePlanner,
      qrFootholdPlanner *footholdPlanner,
      qrUserParameters &userParameters,
      std::string configFilepath);

  /**
   * @brief 默认构造函数 TorqueStanceLegController;
   */
  TorqueStanceLegController() = default;

  /**
   * @brief 默认析构函数 TorqueStanceLegController.
   */
  virtual ~TorqueStanceLegController() = default;

  /**
   * @brief 将期望状态命令绑定到这个stance腿控制器上。
   * @param desiredStateCommandIn
   */
  void BindCommand(qrDesiredStateCommand *desiredStateCommandIn) { desiredStateCommand = desiredStateCommandIn; }

  /**
   * @brief 使用当前时间重置stance腿控制器。
   * @param currentTime: 当前时间重置。
   */
  virtual void Reset(float currentTimeIn);

  /**
   * @brief 更新摩擦力与机器人重力之间的比率。
   * @param contacts: 描述四个脚部接触状态
   * @param N: 接触脚部数量
   * @param normalizedPhase:摆动腿相位
   */
  void UpdateFRatio(Vec4<bool> &contacts, int &N, float &normalizedPhase);

  /**
   * @brief 更新期望命令，特别是使用KP/KD方法的加速度命令。
   * MPC 不使用加速度计算。
   */
  virtual void UpdateDesCommand();

  /**
   * @brief 使用当前时间每个控制循环更新stance腿控制器。
   * @param currentTime: 当前时间更新。
   */
  void Update(float currentTime);

  /**
   * @brief 获取stance腿控制器的电机命令
   * @return 由力平衡计算的命令和力矩阵。
   */
  virtual std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

/**
   * @brief 四足机器人的期望线速度，由用户命令。
   */
  Vec3<float> desiredSpeed;

  /**
   * @brief 四足机器人的期望扭转速度，由用户命令。
   */
  float desiredTwistingSpeed;

  // private:

  /**
   * @brief 存储当前时间。
   */
  float currentTime;

  /**
   * @brief 机器人指针。
   */
  qrRobot *robot;

  /**
   * @brief 步态生成器指针。
   */
  qrGaitGenerator *gaitGenerator;

  /**
   * @brief 机器人状态估计器指针。
   */
  qrRobotEstimator *robotEstimator;

  /**
   * @brief 地面状态估计器指针。
   */
  qrGroundSurfaceEstimator *groundEstimator;

  /**
   * @brief COM 调整器指针。
   */
  qrComAdjuster *comAdjuster;

  /**
   * @brief 姿态规划器指针。
   */
  qrPosePlanner *posePlanner;

  /**
   * @brief 脚部规划器指针。
   */
  qrFootholdPlanner *footholdPlanner;

  /**
   * @brief 期望状态命令指针。
   */
  qrDesiredStateCommand *desiredStateCommand;

  /**
   * @brief 蹲踞时的期望身体高度。
   * 在类构造函数中由 userParameters.desiredHeight 覆盖。
   */
  float desiredBodyHeight = 0.3;

  /**
   * @brief 定义脚部和环境之间的相互作用力。
   */
  Vec4<float> frictionCoeffs;

  /**
   * @brief YAML 节点用于STANCE 腿控制器。
   */
  YAML::Node param;

  /**
   * @brief 一个字符串存储控制模式。
   * 蹲踞、步行或高级蹲踞。
   */
  std::string controlModeStr;

  /**
   * @brief 是否在世界坐标系中计算力。
   */
  bool computeForceInWorldFrame;

  /**
   * @brief 当前时间的stance腿数量。
   */
  int N;

  float moveBasePhase;

  /**
   * @brief 4个元素向量存储4个脚部的接触状态。
   */
  Eigen::Matrix<bool, 4, 1> contacts;

  /**
   * @brief KP 向量用于计算加速度。
   */
  Eigen::Matrix<float, 6, 1> KP;

  /**
   * @brief KD 向量用于计算加速度。
   */
  Eigen::Matrix<float, 6, 1> KD;  // for acceleration

  /**
   * @brief KP/KD 计算的最大加速度。
   */
  Eigen::Matrix<float, 6, 1> maxDdq;

  /**
   * @brief KP/KD 计算的最小加速度。
   */
  Eigen::Matrix<float, 6, 1> minDdq;

  /**
   * @brief 加速度的权重。
   */
  Eigen::Matrix<float, 6, 1> accWeight;

  /**
   * @brief 每个腿部的最小力应用比率。
   */
  Vec4<float> fMinRatio;

  /**
   * @brief 每个腿部的最大力应用比率。
   */
  Vec4<float> fMaxRatio;

  /**
   * @brief 一个迭代计数器用于 debug。
   */
  long long count = 0;
};

}  // namespace Quadruped

#endif  // QR_TORQUE_STANCE_LEG_CONTROLLER_H
