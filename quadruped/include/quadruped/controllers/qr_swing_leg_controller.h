/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_RAIBERT_SWING_LEG_CONTROLLER_H
#define QR_RAIBERT_SWING_LEG_CONTROLLER_H

#include "controllers/qr_foot_trajectory_generator.h"
#include "estimators/qr_state_estimator_container.h"
#include "gait/qr_openloop_gait_generator.h"
#include "planner/qr_foothold_planner.h"
#include "robots/qr_motor.h"

namespace Quadruped {

class qrRaibertSwingLegController {
public:
  /**
   * @brief 成员 qrRaibertSwingLegController 的构造函数。
   * @param robot: 指向 Robot 及其派生类的指针。
   * @param gaitGenerator: 指向 GaitGenerator 的指针。
   * @param stateEstimators: 指向 StateEstimatorContainer 的指针。
   * @param FootholdPlanner: 指向 FootholdPlanner 的指针。
   * @param userParameters: 指向 UserParameters 的指针。
   * @param configPath: 配置文件的路径。
   */
  qrRaibertSwingLegController(
      qrRobot *robot,
      qrGaitGenerator *gaitGenerator,
      qrStateEstimatorContainer *stateEstimators,
      qrFootholdPlanner *qrFootholdPlanner,
      qrUserParameters &userParameters,
      std::string configPath);

  ~qrRaibertSwingLegController() = default;

  /**
   * @brief 将所需命令绑定到这个摆动腿控制器。
   * @param desiredStateCommandIn: 指向 DesiredStateCommand 的指针。
   */
  void BindCommand(qrDesiredStateCommand *desired_state_command) { desiredStateCommand = desired_state_command; }

  /**
   * @brief 二次插值函数，用于生成多边形曲线。
   * @param phase: 指定给定的相位在 [0, 1] 中计算轨迹。
   * @param start: 指定摆动腿开始时的足部位置。
   * @param mid:抛物线的中点。
   * @param end 指定摆动腿结束时的足部所需位置。
   * @return 一个浮点数值，带有相位。
   */
  float GenParabola(float phase, float start, float mid, float end);

  /**
   * @brief 生成摆动腿的轨迹。
   * @param inputPhase: 当前的摆动腿相位。
   * @param startPos:摆动腿的起始位置。
   * @param endPos:摆动腿的最终位置。
   * @return 足部位置，如 (x, y, z)。
   */
  Eigen::Matrix<float, 3, 1> GenSwingFootTrajectory(
      float inputPhase, Eigen::Matrix<float, 3, 1> startPos, Eigen::Matrix<float, 3, 1> endPos);

  /**
   * @brief 使用当前时间重置摆动腿控制器。
   * @param currentTime: 当前的时间，以重置。
   */
  void Reset(float currentTime);

  /**
   * @brief 使用当前时间更新摆动腿控制器。
   * @param currentTime: 当前的时间，以更新。
   */
  void Update(float currentTime);

  /**
   * @brief 获取摆动腿电机的位置模式命令。
   */
  std::map<int, Eigen::Matrix<float, 5, 1>> GetAction();

  /**
   * @brief 指向 DesiredStateCommand 的指针。
   */
  qrDesiredStateCommand *desiredStateCommand;

  /**
   * @brief 四足机器人的所需速度。
   * 通常出现在速度模式中。
   */
  Eigen::Matrix<float, 3, 1> desiredSpeed;

  /**
   * @brief 四足机器人的所需角速度。
   * 通常出现在速度模式中。
   */
  float desiredTwistingSpeed;

  /**
   * @brief Raibert公式中的位置校正系数。
   * 在公式中，Kp = sqrt(z0/||g||)。
   * 目前，我们将其设置为：const Matrix<float, 3, 1> swingKp(0.03, 0.03, 0.03)。
   */
  Eigen::Matrix<float, 3, 1> swingKp;

  /**
   * @brief 指向 Robot 的指针。
   */
  qrRobot *robot;

  /**
   * @brief 指向 GaitGenerator 的指针。
   */
  qrGaitGenerator *gaitGenerator;

  /**
   * @brief 指向 RobotEstimator 的指针。
   */
  qrRobotEstimator *stateEstimator;

  /**
   * @brief 指向 GroundSurfaceEstimator 的指针。
   */
  qrGroundSurfaceEstimator *groundEstimator;

  /**
   * @brief 指向 FootholdPlanner 的指针。
   */
  qrFootholdPlanner *footholdPlanner;

  /**
   * @brief 指向 UserParameters 的指针。
   */
  qrUserParameters *userParameters;

  /**
   * @brief 存储摆动腿的腿id。
   */
  std::vector<u8> swingFootIds;

  /**
   * @brief 上一个控制循环的腿状态。
   */
  Eigen::Matrix<int, 4, 1> lastLegState;

  /**
   * @brief 四足机器人的所需高度。
   */
  Eigen::Matrix<float, 3, 1> desiredHeight;

  /**
   * @brief 存储所需的关节角度和速度。
   * 从关节ID到关节角度、速度和腿ID的映射。
   */
  std::map<int, std::tuple<float, float, int>> swingJointAnglesVelocities;

  /**
   * @brief 在换腿状态时的足部位置（在基坐标系中）。
   */
  Eigen::Matrix<float, 3, 4> phaseSwitchFootLocalPos;

  /**
   * @brief 在换腿状态时的足部位置（在世界坐标系中）。
   */
  Eigen::Matrix<float, 3, 4> phaseSwitchFootGlobalPos;

  /**
   * @brief 足部支撑点（在世界坐标系中）。
   */
  Eigen::Matrix<float, 3, 4> footHoldInWorldFrame;

  /**
   * @brief 所需的足部位置（在世界坐标系中）。
   * 这个成员只用于 Gazebo，可能在将来被删除。
   */
  Eigen::Matrix<float, 3, 4> footTargetPositionsInWorldFrame;

  /**
   * @brief 所需的足部位置（在基坐标系中）。
   */
  Eigen::Matrix<float, 3, 4> desiredFootPositionsInBaseFrame;

  /**
   * @brief 存储所有 4 条腿的轨迹对象。
   * 将在任何一腿摆动时使用。
   */
  SwingFootTrajectory swingFootTrajectories[4];

  /**
   * @brief摆动轨迹的样条信息。
   */
  qrSplineInfo splineInfo;

  /**
   * @brief swing_leg_controller.yaml 的配置文件路径。
   */
  YAML::Node swingLegConfig;

  /**
   * @brief 足部偏移量。这将在将来被删除。
   */
  float footOffset;

  /**
   * @brief 上一个控制循环的足部位置（在基坐标系中）。
   */
  Eigen::Matrix<float, 3, 4> foot_pos_target_last_time;

  /**
   * @brief 与 foot_pos_target_last_time 相同。将在将来被删除。
   */
  Eigen::Matrix<float, 3, 4> foot_pos_rel_last_time;

  Eigen::Matrix<float, 3, 4> foot_forces_kin;
  Eigen::Matrix<float, 3, 4> foot_pos_error;
  Eigen::Matrix<float, 3, 4> foot_vel_error;
};

}  // namespace Quadruped

#endif  // QR_RAIBERT_SWING_LEG_CONTROLLER_H
