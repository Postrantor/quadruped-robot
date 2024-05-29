/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_COM_ADJUSTER_H
#define QR_COM_ADJUSTER_H

#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "estimators/qr_robot_estimator.h"
#include "gait/qr_openloop_gait_generator.h"
#include "robots/qr_robot.h"
#include "utils/qr_se3.h"

namespace Quadruped {

/**
 * 计划中心质量（CoM）的路径，使得中心质量（CoM）落入多边形内部
 */
class qrComAdjuster {
private:
  /**
   * @brief ADJEST_LEG 向量索引表示腿的顺序，
   *        ADJEST_LEG 的值表示相邻两个腿的顺序，
   *        cw 表示顺时针腿，ccw 表示逆时针腿。
   */
  const std::vector<std::map<std::string, int>> ADJEST_LEG{
      std::map<std::string, int>{{"cw", 2}, {"ccw", 1}}, std::map<std::string, int>{{"cw", 0}, {"ccw", 3}},
      std::map<std::string, int>{{"cw", 3}, {"ccw", 0}}, std::map<std::string, int>{{"cw", 1}, {"ccw", 2}}};

  /**
   * @brief 机器人类用于 CoM 计划。
   */
  qrRobot *robot;

  /**
   * @brief 步态生成器对象。
   */
  qrGaitGenerator *gaitGenerator;

  /**
   * @brief 机器人估算器对象。
   */
  qrRobotEstimator *robotEstimator;

  /**
   * @brief 机器人状态中的 CoM 位置。
   */
  Eigen::Vector3f basePosition;

  /**
   * @brief 机器人状态中的 CoM 姿态。
   */
  Eigen::Vector4f baseOrientation;

  /**
   * @brief 在基坐标系中调整后的 CoM 位置。
   */
  Eigen::Matrix<float, 3, 1> comPosInBaseFrame;

  /**
   * @brief 在世界坐标系中调整后的 CoM 位置。
   */
  Eigen::Matrix<float, 3, 1> comPosInWorldFrame;

  /**
   * @brief 每个腿的状态来自 qrGaitGenerator 类，
   *        例如 SWING/STAND。
   */
  Eigen::Matrix<int, 4, 1> legState;

  /**
   * @brief 所需状态的相对相位。
   */
  Eigen::Matrix<float, 4, 1> normalizedPhase;

  /**
   * @brief 机器人状态中的脚端位置在基坐标系中。
   */
  Eigen::Matrix<float, 3, 4> footPosition;  // 在基坐标系中

  /**
   * @brief 脚端与地面接触的概率（范围：0.~1.）。
   */
  float contactK[4];  // 是否脚端与地面接触。

  /**
   * @brief 每个腿是否是摆动腿的概率。
   */
  float swingK[4];  // 是否是摆动腿？

  /**
   * @brief 每个腿的接触概率。
   * 即 weightFactor[legId] = contactK[legId] + swingK[legId]
   */
  float weightFactor[4];  // 顶点权重。

  /**
   * @brief 支持多边形的坐标。
   */
  Eigen::Matrix<float, 3, 4> supportPolygonVertices;

  /**
   * @brief 用于计算 contactK 和 swingK。
   */
  float delta;

public:
  /**
   * @brief ComAdjuster 类的构造函数。
   * @param robotIn: 机器人用于 CoM 计划。
   * @param gaitGeneratorIn: 生成所需步态。
   * @param robotEstimatorIn: 估算 CoM 姿态和位置。
   */
  qrComAdjuster(qrRobot *robotIn, qrGaitGenerator *gaitGeneratorIn, qrRobotEstimator *robotEstimatorIn);

  /**
   * @brief ComAdjuster 类的析构函数。
   */
  ~qrComAdjuster() = default;

  /**
   * @brief 在控制器启动时调用。
   * @param currentTime:墙面时间，以秒为单位。
   */
  void Reset(float currentTime);

  /**
   * @brief 更新接触状态和虚拟多边形。
   * 虚拟支持多边形偏离即将结束接触相位的腿，
   * 并偏向即将触摸的腿。
   * @param currentTime:墙面时间，以秒为单位。
   * @return 所需 CoM 位置
   */
  Eigen::Matrix<float, 3, 1> Update(float currentTime);

  /**
   * @brief 获取 comPosInBaseFrame 成员的 getter 方法。
   */
  inline Eigen::Matrix<float, 3, 1> &GetComPosInBaseFrame() { return comPosInBaseFrame; };
};

}  // Namespace Quadruped

#endif  // QR_COM_ADJUSTER_H
