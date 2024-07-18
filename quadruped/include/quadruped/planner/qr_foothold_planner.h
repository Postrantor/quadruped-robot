/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_Foothold_PLANNER_H
#define QR_Foothold_PLANNER_H

#include "quadruped/fsm/qr_control_fsm_data.hpp"
#include "quadruped/planner/qr_foot_stepper.h"

namespace Quadruped {

/**
 * @brief 计划下一个摆动阶段的脚部placement。
 */
class qrFootholdPlanner {
public:
  /**
   * @brief qrFootholdPlanner 的构造函数。
   * @param robotIn 机器人对象指针。
   * @param groundEstimator 地面估算器。
   */
  qrFootholdPlanner(
      qrRobot *quadrupedIn,
      qrGaitGenerator *gaitGeneratorIn,
      qrStateEstimatorContainer *stateEstimatorsIn,
      qrUserParameters *userParametersIn,
      qrDesiredStateCommand *desiredStateCommandIn);

  /**
   * @brief qrFootholdPlanner 对象的析构函数。
   */
  ~qrFootholdPlanner() = default;

  /**
   * @brief 重置 foothold 规划器。
   */
  void Reset(float t);

  /**
   * @brief 更新 foothold 规划器。
   */
  void Update() {}

  /**
   * @brief 只在抬腿前一刻调用。
   */
  void UpdateOnce(Eigen::Matrix<float, 3, 4> currentFootholds, std::vector<int> legIds = {});

  /**
   * @brief 计算走模式下的期望脚端位置。
   * @param currentFootholds 当前所有腿的脚端位置。
   * @param currentComPose 当前 COM 位移和姿态。
   * @param desiredComPose 期望 COM 位移和姿态。
   * @param legIds 腿的顺序。
   */
  Eigen::Matrix<float, 3, 4> ComputeNextFootholds(
      Eigen::Matrix<float, 3, 4> &currentFootholds,
      Eigen::Matrix<float, 6, 1> &currentComPose,
      Eigen::Matrix<float, 6, 1> &desiredComPose,
      std::vector<int> &legIds);

  /**
   * @brief 计算位置模式下的脚端位置增量。
   * @param currentFootholds 当前所有腿的脚端位置。
   */
  Eigen::Matrix<float, 3, 4> ComputeFootholdsOffset(
      Eigen::Matrix<float, 3, 4> currentFootholds,
      Eigen::Matrix<float, 6, 1> currentComPose,
      Eigen::Matrix<float, 6, 1> desiredComPose,
      std::vector<int> legIds);

  /**
   * @brief 获取期望 COM 位移和姿态。
   */
  inline const Eigen::Matrix<float, 6, 1> &GetDesiredComPose() const { return desiredComPose; }

  /**
   * @brief 获取期望脚端位置增量。
   */
  inline const Eigen::Matrix<float, 3, 4> &GetFootholdsOffset() const { return desiredFootholdsOffset; }

  /**
   * @brief 获取期望 COM 位移和姿态。
   */
  inline Eigen::Matrix<float, 6, 1> GetComGoal(Eigen::Matrix<float, 6, 1> currentComPose) {
    desiredComPose << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;
    return desiredComPose;
  };

  /**
   * @brief 获取世界坐标系下的脚部placement。
   */
  inline Vec3<float> GetFootholdInWorldFrame(int legId) { return desiredFootholds.col(legId); };

  /**
   * @brief 计算期望脚部placement。
   * @param swingFootIds: 摆动腿的id。
   */
  void ComputeHeuristicFootHold(std::vector<u8> swingFootIds);

  /**
   * @brief 使用 MIT 方法计算期望脚部placement。
   * @param swingFootIds: 摆动腿的id。
   */
  void ComputeMITFootHold(int legId);

public:
  /**
   * @brief 机器人对象。
   */
  qrRobot *robot;

  /**
   * @brief 步态生成器对象。
   */
  qrGaitGenerator *gaitGenerator;

  /**
   * @brief 地面估计器对象。
   */
  qrGroundSurfaceEstimator *groundEsitmator;

  /**
   * @brief 一些计算参数，包括 EKF、地形、滑动窗口等。
   */
  qrUserParameters *userParameters;

  /**
   * @brief 期望状态命令。
   */
  qrDesiredStateCommand *desiredStateCommand;

  /**
   * @brief qrFootStepper 对象。
   */
  qrFootStepper *footstepper;

  /**
   * @brief 重置时的时间。
   */
  float resetTime;

  /**
   * @brief 重置时的时间 для footStepper。
   */
  float timeSinceReset;

  /**
   * @brief footStepper 的配置。
   */
  YAML::Node footStepperConfig;

  /**
   * @brief 缺口的宽度。
   */
  float gapWidth;

  /**
   * @brief 位置模式下的脚端位置增量。
   */
  float footHoldOffset;

  /**
   * @brief 地图上的缺口信息。
   */
  std::vector<float> gaps;

  /**
   * @brief 地形信息。
   */
  qrTerrain &terrain;

  /**
   * @brief 默认的地图尺寸。
   */
  constexpr static int N = 50;  // 默认的地图尺寸

  /**
   * @brief 当前的 COM 位移和姿态。
   */
  Eigen::Matrix<float, 6, 1> comPose;

  /**
   * @brief 期望的 COM 位移和姿态。
   */
  Eigen::Matrix<float, 6, 1> desiredComPose;

  /**
   * @brief 期望的脚端位置增量。
   */
  Eigen::Matrix<float, 3, 4> desiredFootholdsOffset;

  /**
   * @brief 期望的脚端位置。
   */
  Eigen::Matrix<float, 3, 4> desiredFootholds;

  /**
   * @brief 相对相位。
   */
  Vec4<float> phase;

  /**
   * @brief 摆动腿的系数。
   */
  Vec3<float> swingKp;

  /**
   * @brief 机器人下降的步数。
   */
  int moveDown[4] = {0, 0, 0, 0};

  /**
   * @brief 首次摆动的基础状态。
   */
  Vec12<float> firstSwingBaseState;
};

}  // namespace Quadruped

#endif  // QR_FOOTHOLD_PLANNER_H
