/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_GROUND_ESTIMATOR_H
#define QR_GROUND_ESTIMATOR_H

#include "estimators/qr_base_state_estimator.h"
#include "robots/qr_robot.h"
#include "utils/qr_se3.h"

namespace Quadruped {

/**
 * @brief 一种地形： ground----| gap |----ground
 */
struct qrGap {
  /**
   * @brief COM 到gap中心的距离。
   */
  float distance;
  /**
   * @brief gap 的宽度。
   */
  float width;

  /**
   * @brief 基坐标系中gap 边缘的最近点。
   */
  Eigen::Matrix<float, 3, 1> startPoint;

  /**
   * @brief gap 结构体的构造函数。
   * @param d: COM 到gap中心的距离
   * @param w: gap 的宽度
   * @param p: 基坐标系中gap 边缘的最近点。
   */
  inline qrGap(float d, float w, Eigen::Matrix<float, 3, 1> p) : distance(d), width(w), startPoint(p) {}
};

/**
 * @brief 一种地形。
 */
struct qrStair {
  /**
   * @brief 阶梯的高度。
   */
  float height;

  /**
   * @brief 阶梯的宽度。
   */
  float width;

  /**
   * @brief 阶梯的长度。
   */
  float length = 1.0;

  /**
   * @brief 基坐标系中gap 边缘的最近点。
   */
  Eigen::Matrix<float, 3, 1> startPoint;  // 基坐标系中gap 边缘的最近点。

  /**
   * @brief 阶梯的数量。
   */
  int k = 3;

  /**
   * @brief 结构体 Stair 的默认构造函数。
   */
  inline qrStair() = default;

  /**
   * @brief 阶梯结构体的构造函数。
   * @param h: 阶梯的高度。
   * @param w: 阶梯的宽度。
   * @param l: 阶梯的长度。
   * @param p: 基坐标系中gap 边缘的最近点。
   */
  inline qrStair(float h, float w, float l, Eigen::Matrix<float, 3, 1> p)
      : height(h), width(w), length(l), startPoint(p){};
};

/**
 * @brief 地形结构体。
 */
struct qrTerrain {
  /**
   * @brief 地形的类型。
   */
  TerrainType terrainType;

  /**
   * @brief 足部接触点偏移量到足部关节。
   */
  float footHoldOffset = 0.1f;

  /**
   * @brief gap 的 vector。
   */
  std::vector<qrGap*> gaps;

  /**
   * @brief 指向阶梯的指针。
   */
  qrStair* stair;

  /**
   * @brief 足部规划器的成本地图矩阵。
   */
  Eigen::MatrixXf costMap;
};

/**
 * @brief 按照 MIT CHEETAH3 论文所述，3D 平面为 z(x,y) = a0+ a1*x +a2*y
 * @param a  Vec3<float>, 地面表面平面的系数，z= a0+a1*x+a2*y
 */
class qrGroundSurfaceEstimator : public qrBaseStateEstimator {
public:
  /**
   * @brief 地面估算器的构造函数。
   * @param Robot: 机器人类用于地面估算。
   * @param terrainConfigPath: 地形配置文件的路径。
   */
  qrGroundSurfaceEstimator(qrRobot* robot, std::string terrainConfigPath);

  /**
   * @brief 加载地形配置文件。
   * @param terrainConfigPath: 地形配置文件的路径。
   */
  void Loadterrain(std::string& terrainConfigPath);

  /**
   * @brief 重置估算器。
   * @param currentTime: 计时器启动以来的时间。
   */
  virtual void Reset(float currentTime);

  /**
   * @brief 当四个脚都接触地面时，计算平面方程。
   * @param currentTime: 计时器启动以来的时间。
   */
  virtual void Update(float currentTime);

  /**
   * @brief 计算或返回地面表面的法向量，表示在（初始）基坐标系中。
   * @param update: 是否 normalize 法向量。
   * @return 地面表面的法向量，表示在（初始）基坐标系中。
   */
  Eigen::Matrix<double, 3, 1> GetNormalVector(bool update);

  /**
   * @brief 控制帧是 COM 原点的帧，x 轴与身体 COM 帧的 X 轴对齐，同时其 z 轴垂直于估算的地面表面，y
   * 轴平行于局部表面平面。
   * @return 控制帧相对于世界帧的同质变换矩阵。
   */
  Eigen::Matrix<double, 4, 4> ComputeControlFrame();

  /**
   * @brief 获取基坐标系中某点的地面高度。
   * @param x: 基坐标系中的 x 坐标。
   * @param y: 基坐标系中的 y 坐标。
   * @return 地面高度。
   */
  float GetZInBaseFrame(float x, float y);

  /**
   * @brief 获取当前控制帧中某点的地面高度，通过给定的 x 和 y 坐标在最后控制帧中。
   * @param x: 上一个控制帧中的 x 坐标。
   * @param y: 上一个控制帧中的 y 坐标。
   * @return 当前控制帧中的地面高度。
   */
  float GetZInControlFrame(float x, float y);

  /**
   * @brief 获取当计算 GRF 时世界坐标系中的三个方向向量。
   * @return 控制帧相对于世界帧的旋转矩阵。
   */
  Eigen::Matrix<float, 3, 3> GetAlignedDirections();

  /**
   * @brief 获取世界坐标系中的控制帧朝向四元数。
   * @return 控制帧相对于世界帧的朝向四元数。
   */
  Quat<float> GetControlFrameOrientation() const { return controlFrameOrientation.cast<float>(); };

  /**
   * @brief 获取世界坐标系中的控制帧 roll-pitch-yaw。
   * @return 控制帧相对于世界帧的 roll-pitch-yaw。
   */
  Vec3<float> GetControlFrameRPY() const { return controlFrameRPY.cast<float>(); };

  /**
   * @brief 控制帧相对于世界帧的 roll-pitch-yaw。
   */
  Vec3<double> controlFrameRPY;

  /**
   * @brief 控制帧相对于世界帧的朝向四元数。
   */
  Quat<double> controlFrameOrientation;

  /**
   * @brief 地面估算中的机器人类。
   */
  qrRobot* robot;

  /**
   * @brief 机器人行走的地形。
   */
  qrTerrain terrain;

  /**
   * @brief 平面方程的系数。
   */
  Vec3<double> a;

  /**
   * @brief W 包含四个脚部的位置，
   * 用于计算平面方程的系数。
   * W = [1, p_x, p_y]_4*3, p 包含每个腿部的数据，p_x = [p_x1, p_x2, p_x3, p_x4]
   */
  Eigen::Matrix<double, 4, 3> W;

  /**
   * @brief 基坐标系中四个脚部的 Z 位置。
   */
  Vec4<double> pZ;

  /**
   * @brief 基坐标系中平面方程的法向量。
   */
  Vec3<double> n;

  /**
   * @brief 机器人身体在世界坐标系中的位置。
   */
  Vec3<float> bodyPositionInWorldFrame;

  /**
   * @brief 控制帧相对于世界帧的同质变换矩阵。
   */
  Mat4<double> controlFrame;

  /**
   * @brief 上一个控制循环中四个腿部的接触状态。
   */
  Eigen::Matrix<bool, 4, 1> lastContactState;

  /**
   * @brief 足部规划器配置文件的 YAML 节点。
   */
  YAML::Node footStepperConfig;
};

}  // Namespace Quadruped

#endif  // QR_GROUND_ESTIMATOR_H
