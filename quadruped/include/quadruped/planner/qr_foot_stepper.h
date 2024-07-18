/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR__FOOT_STEPPER_H
#define QR__FOOT_STEPPER_H

#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <tuple>
#include <vector>

#include "Array.hh"
#include "QuadProg++.hh"
#include "estimators/qr_state_estimator_container.h"
#include "robots/qr_robot.h"
#include "utils/qr_se3.h"

namespace Quadruped {

/**
 * @brief FootStepper 类
 * @todo 不同的 terrain 可能需要不同的 FootStepper，使用工厂方法。
 */
class qrFootStepper {
public:
  /**
   * @brief FootStepper 类的构造函数。
   * @param terrain: 机器人接触的 3D 平面。
   * @param defaultFootholdOffset: 默认的脚端偏移量。
   * @param level: 地形的级别。
   */
  qrFootStepper(qrTerrain& terrain, float defaultFootholdOffset, std::string level);

  void Reset(float timeSinceReset){};

  /**
   * @brief 获取默认的脚端偏移量。
   * @param legId: 哪条腿。
   * @return 默认的脚端偏移量。
   */
  inline Eigen::Matrix<float, 3, 1> GetDefaultFootholdOffset(int legId) {
    return {defaultFootholdDelta, 0.f, 0.f};  // todo : 1 DIM
  };

  /**
   * @brief 查找最佳的脚部placement，通常大于零。
   * @param Eigen::Matrix<float, 3, 4> 脚部位置在世界坐标系中的位置。
   * @note 假设脚部偏移量 L = L0 + x，_gap 宽度是 W，
   *       成本目标是 F = x^T * G * x + a^T * x = x^2，
   *       这意味着我们想要默认偏移量的增量尽可能小。
   *       约束不等式是 :
   *           C^T * x >= b
   *       情况 1：如果前腿可以以默认偏移量跨过 gap，
   *       那么 x 应该满足条件：L0 - d(foot, center of gap) + x >= W/2 或 <=-W/2;
   *       这意味着前腿要么（1.a）跨过 gap，要么（1.b）不跨过 gap。
   *       同时，后腿不跨过 gap。
   *       要表达（1.b）在矩阵形式中，
   *           [1, -1, -1, -1, -1]^T * x = [x >= b = [ -L0
   *                                       -x
   *                                       -x           L0-p(gap)+p(foot)
   *                                       -x
   *                                       -x]         ]
   */
  Eigen::Matrix<float, 3, 4> GetOptimalFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds);

  /**
   * @brief 计算下一个脚部placement 在世界坐标系中的位置。
   * @param currentFootholds: 当前的脚部placement 在世界坐标系中的位置。
   * @param currentComPose: 当前的 CoM 姿态。
   * @param desiredComPose: 希望的 CoM 姿态。
   * @param legIds: 腿 Id。
   * @return 下一个脚部placement 在世界坐标系中的位置。
   */
  std::tuple<Eigen::Matrix<float, 3, 4>, Eigen::Matrix<float, 3, 4>> GetFootholdsInWorldFrame(
      Eigen::Matrix<float, 3, 4>& currentFootholds,
      Eigen::Matrix<float, 6, 1>& currentComPose,
      Eigen::Matrix<float, 6, 1>& desiredComPose,
      std::vector<int>& legIds);

  /**
   * @brief 查找最佳的脚部placement，并检查它。
   * @param currentFootholdsX: 当前的脚部placement 的 x 轴位置。
   * @param front: 是否前腿跨过 gap，1.0： true，-1.0： false。
   * @param back: 是否后腿跨过 gap。
   * @param frontGap: 前 gap。
   * @param backGap: 后 gap。
   * @return 如果解决方案有效，返回 x; 否则，返回 -1;
   */
  double CheckSolution(
      Eigen::Matrix<float, 1, 4> currentFootholdsX, double front, double back, qrGap frontGap, qrGap backGap);

  /**
   * @brief 生成下一个步骤偏移量沿 x 轴 untuk 腿跨过障碍。
   * @return int, 0: 成功, -1: 调整之前的步骤, -2: 无效解决方案。
   */
  int StepGenerator(Eigen::Matrix<float, 1, 4>& currentFootholds, Eigen::Matrix<float, 1, 4>& desiredFootholdsOffset);

protected:
  /**
   * @brief 缝隙的大小。
   */
  std::vector<qrGap> gaps;

  /**
   * @brief 描述楼梯信息的 map。
   */
  qrStair stairUp, stairDown;

  /**
   * @brief 包含未来步骤偏移量沿 x 轴 untuk 腿跨过障碍。
   */
  std::queue<Eigen::Matrix<float, 1, 4>> steps;  // 包含未来步骤偏移量沿 x 轴 untuk 腿跨过障碍。

  /**
   * @brief 如果 false，表示还没有生成步骤。
   */
  bool generatorFlag = false;  // 如果 false，表示还没有生成步骤

  /**
   * @brief 如果 true，表示已经改变了步行方式。
   */
  bool gaitFlag = false;  // 如果 true，表示已经改变了步行方式

  /**
   * @brief 是否任何脚部跨过 gap。
   */
  bool meetGpa;  // 是否任何脚部跨过 gap？

  /**
   * @brief 默认的脚端偏移量。
   */
  float defaultFootholdDelta;

  /**
   * @brief 下一个步骤的脚端偏移量。
   */
  Eigen::Matrix<float, 3, 4> nextFootholdsOffset;

  /**
   * @brief 上一个步骤的脚端偏移量。
   */
  Eigen::Matrix<float, 3, 4> lastFootholdsOffset;

  /**
   * @brief Z 轴上的偏移量。
   */
  Vec4<float> dZ;

  /**
   * @brief QP 问题的参数：
   * min 0.5* x G x + g0 x
   * s.t.
   * CE^T x + ce0 = 0
   * CI^T x + ci0 >= 0
   */
  quadprogpp::Matrix<double> G;
  quadprogpp::Vector<double> a;
  quadprogpp::Matrix<double> CE;
  quadprogpp::Vector<double> e;
  quadprogpp::Matrix<double> CI;
  quadprogpp::Vector<double> b;

  /**
   * @brief QP 问题的解决方案。
   */
  quadprogpp::Vector<double> x;
};

}  // Namespace Quadruped

#endif  // QR_FOOT_STEPPER_H
