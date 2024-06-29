/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_FOOT_TRAJECTORY_GENERATOR_H
#define QR_FOOT_TRAJECTORY_GENERATOR_H

#include <glm/glm.hpp>
#include <tinynurbs/tinynurbs.h>

#include <Eigen/Dense>

#include "config.h"
#include "utils/qr_geometry.h"
#include "utils/qr_tools.h"

namespace Quadruped {

enum SplineType { XYLinear_ZParabola = 0, QuadraticPolygon, CubicPolygon, QuinticPolygon, BSpline };

struct qrStepParameters {
  qrStepParameters() = default;

  qrStepParameters(float _duration, float _height, float _penetration = 0.)
      : duration(_duration), height(_height), penetration(_penetration) {}

  /**
   * @brief 步长的持续时间。
   */
  float duration = 0.0f;

  /**
   * @brief 步长的高度。
   */
  float height = 0.0f;

  /**
   * @brief 摆动轨迹的穿透距离。
   */
  float penetration = 0.0f;
};

struct qrSplineInfo {
  qrSplineInfo() = default;

  /**
   * @brief 构造函数。
   * @param control_points:样条曲线的控制点。
   * @param knots:样条曲线的节点。
   */
  qrSplineInfo(std::vector<glm::vec3> &control_points, std::vector<float> &knots) {
    this->splineType = SplineType::BSpline;
    this->controlPoints = control_points;
    this->knots = knots;
  }

  /**
   * @brief 样条曲线的度数，例如 2、3、5。
   */
  int degree = 3;

  /**
   * @brief 样条曲线的类型。选项值：quadratic、cubicPolygon、quinticPolygon、BSpline。
   */
  SplineType splineType = SplineType::CubicPolygon;

  std::vector<glm::vec3> controlPoints;

  std::vector<float> knots;
};

class qrFootSplinePatternGenerator {
public:
  qrFootSplinePatternGenerator() = default;

  virtual ~qrFootSplinePatternGenerator() = default;

  /**
   * @brief 设置足部摆动轨迹的参数。
   * 这个方法假设轨迹中没有障碍物。
   * 这个方法是一个纯虚拟函数。任何用户定义的样条曲线都应该实现这个函数。
   * @param initial_time: 轨迹的开始时间。
   * @param initial_pos: 足部的初始位置。
   * @param target_pos: 本周期的目标足部位置。
   * @param params: 样条曲线的其他参数，@see Quadruped::StepParameters。
   */
  virtual void SetParameters(
      const float initial_time,
      const Eigen::Vector3f &initial_pos,
      const Eigen::Vector3f &target_pos,
      const qrStepParameters &params) = 0;

  /**
   * @brief 生成足部摆动轨迹。
   * @param foot_pos: 足部的瞬时位置。
   * @param foot_vel: 足部的瞬时速度。
   * @param foot_acc: 足部的瞬时加速度。
   * @param time: 当前时间。
   */
  virtual bool GenerateTrajectory(
      Eigen::Vector3f &foot_pos, Eigen::Vector3f &foot_vel, Eigen::Vector3f &foot_acc, float time) = 0;

  /**
   * @brief 每个控制循环更新样条曲线。
   * @param initial_time: 更新样条曲线的初始时间。
   * @param duration: 样条曲线的持续时间。
   * @param initial_pos: 更新样条曲线的初始 foothold 位置。
   * @param target_appex
   * @param target_pos: 样条曲线的目标位置。
   */
  virtual void UpdateSpline(
      float initial_time,
      float duration,
      Eigen::Vector3f &initial_pos,
      float target_appex,
      Eigen::Vector3f &target_pos) {}

protected:
  /**
   * @brief 摆动轨迹的初始时间。
   */
  float initialTime = 0.0f;

  /**
   * @brief 摆动轨迹的持续时间。
   */
  float duration = 0.0f;

  /**
   * @brief 样条曲线的起始位置。
   */
  Vec3<float> startPos;

  /**
   * @brief 样条曲线的终止位置。
   */
  Vec3<float> endPos;

  /**
   * @brief 转换世界坐标系到规范坐标系的旋转矩阵。
   */
  Mat3<float> RTheta;

  /**
   * @brief 样条曲线的初始位置。
   */
  Vec3<float> Tp;
};

class qrFootParabolaPatternGenerator : public qrFootSplinePatternGenerator {
public:
  qrFootParabolaPatternGenerator() = default;

  virtual ~qrFootParabolaPatternGenerator() = default;

  /**
   * @see Quadruped::FootSplinePatternGenerator
   */
  virtual void SetParameters(
      const float initial_time,
      const Eigen::Vector3f &initial_pos,
      const Eigen::Vector3f &target_pos,
      const qrStepParameters &params);

  /**
   * @see Quadruped::FootSplinePatternGenerator
   */
  virtual bool GenerateTrajectory(
      Eigen::Vector3f &foot_pos, Eigen::Vector3f &foot_vel, Eigen::Vector3f &foot_acc, float time);

  /**
   * @see Quadruped::FootSplinePatternGenerator
   */
  virtual void UpdateSpline(
      float initial_time,
      float duration,
      Eigen::Vector3f &initial_pos,
      float target_appex,
      Eigen::Vector3f &target_pos) {}

private:
  /**
   * @brief 足部运动的不同轴 spliner
   */
  robotics::math::qrQuadraticSpline footSplinerZ;

  /**
   * @brief 步伐参数用于 spline 更新
   */
  qrStepParameters stepParameters;
};

class qrFootCubicPatternGenerator : public qrFootSplinePatternGenerator {
public:
  qrFootCubicPatternGenerator() = default;

  virtual ~qrFootCubicPatternGenerator() = default;

  /**
   * @see Quadruped::FootSplinePatternGenerator
   */
  virtual void SetParameters(
      const float initial_time,
      const Eigen::Vector3f &initial_pos,
      const Eigen::Vector3f &target_pos,
      const qrStepParameters &params);

  /**
   * @see Quadruped::FootSplinePatternGenerator
   */
  virtual bool GenerateTrajectory(
      Eigen::Vector3f &foot_pos, Eigen::Vector3f &foot_vel, Eigen::Vector3f &foot_acc, float time);

  /**
   * @see Quadruped::FootSplinePatternGenerator
   */
  virtual void UpdateSpline(
      float initial_time,
      float duration,
      Eigen::Vector3f &initial_pos,
      float target_appex,
      Eigen::Vector3f &target_pos) {}

private:
  /**
   * @brief 足部运动 x 轴的 spliner
   */
  robotics::math::qrCubicSpline footSplinerX;

  /**
   * @brief 足部运动 y 轴的 spliner
   */
  robotics::math::qrCubicSpline footSplinerY;

  /**
   * @brief 足部运动 z-up 轴的 spliner
   */
  robotics::math::qrCubicSpline footSplinerUpZ;

  /**
   * @brief 足部运动 z-down 轴的 spliner
   */
  robotics::math::qrCubicSpline footSplinerDownZ;
};

class qrFootBSplinePatternGenerator : public qrFootSplinePatternGenerator {
public:
  /**
   * @brief 构造函数，初始化 qrFootBSplinePatternGenerator 类。
   * @param splinInfo: BSpline 信息。
   */
  qrFootBSplinePatternGenerator(qrSplineInfo &splineInfo);

  virtual ~qrFootBSplinePatternGenerator() = default;

  /**
   * @see Quadruped::FootSplinePatternGenerator
   */
  virtual void SetParameters(
      const float initial_time,
      const Eigen::Vector3f &initial_pos,
      const Eigen::Vector3f &target_pos,
      const qrStepParameters &params);

  /**
   * @see Quadruped::FootSplinePatternGenerator
   */
  virtual bool GenerateTrajectory(
      Eigen::Vector3f &foot_pos, Eigen::Vector3f &foot_vel, Eigen::Vector3f &foot_acc, float time);

  /**
   * @see Quadruped::FootSplinePatternGenerator
   */
  virtual void UpdateSpline(
      float initial_time,
      float duration,
      const Eigen::Vector3f &initial_pos,
      float target_appex,
      const Eigen::Vector3f &target_pos);

private:
  tinynurbs::Curve3f crv;

  std::vector<glm::vec3> controlPointsTemplate;
};

class SwingFootTrajectory {
public:
  SwingFootTrajectory() = default;

  /**
   * @brief 构造函数，初始化 qrSwingFootTrajectory 类。
   * @param splineInfoIn: BSplineInfo 如果需要。
   * @param startPosIn: 足部swingleg 的起始位置。
   * @param endPosIn: 足部swingleg 的结束位置。
   * @param duration: 轨迹的持续时间，一般是swing 持续时间。
   * @param maxClearance: 轨迹的最高点。
   */
  SwingFootTrajectory(
      qrSplineInfo splineInfoIn,
      Vec3<float> startPosIn = {0.f, 0.f, 0.f},
      Vec3<float> endPosIn = {0.f, 0.f, 0.f},
      float duration = 1.f,
      float maxClearance = 0.1f);

  SwingFootTrajectory(const SwingFootTrajectory &item);

  virtual ~SwingFootTrajectory() = default;

  /**
   * @brief 每次需要控制点时调用。
   * @param Vec3<float>& 足部位置;
   * @param Vec3<float>& 足部线速度;
   * @param Vec3<float>& 足部加速度;
   * @param float 时间相位，范围 [0,1]。
   * @param bool 是否需要模块相位， default -<em> false </em>。
   * @return bool 标志是否正确。
   */
  bool GenerateTrajectoryPoint(
      Vec3<float> &footPos, Vec3<float> &footV, Vec3<float> &footA, float t, bool phaseModule = false);

  /**
   * @brief 新的swing 足部轨迹开始。
   */
  void ResetFootTrajectory(
      float duration, const Vec3<float> &initialPos, const Vec3<float> &targetPos, float height = 0.15f) {
    stepParams.duration = duration;
    stepParams.height = height;
    footTarjGen->SetParameters(0., initialPos, targetPos, stepParams);
  };

  /**
   * @brief 在中空中出错，调整行为。
   */
  void ResetFootTrajectory(float duration, float currentTime, const Vec3<float> &targetPos) {
    stepParams.duration = duration;
    footTarjGen->SetParameters(currentTime, startPos, targetPos, stepParams);
  };

  float mid;
  Vec3<float> startPos;
  Vec3<float> endPos;
  qrStepParameters stepParams;
  qrFootSplinePatternGenerator *footTarjGen;
  qrSplineInfo splineInfo;
};

}  // namespace Quadruped

#endif  // QR_FOOT_TRAJECTORY_GENERATOR_H
