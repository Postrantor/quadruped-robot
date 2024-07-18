/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_STATE_DATAFLOW_H
#define QR_STATE_DATAFLOW_H

#include <yaml-cpp/yaml.h>

#include "quadruped/config/qr_config.h"
#include "quadruped/utils/qr_geometry.h"
#include "quadruped/utils/qr_se3.h"
#include "quadruped/utils/qr_visualization.h"

struct qrUserParameters {
  /**
   * @brief qrUserParameters 类的构造函数。
   * @param file_path: 用户参数文件路径。
   */
  qrUserParameters(std::string file_path);

  /**
   * @brief 用于上楼梯。
   * 将来将被删除。
   */
  float stairsTime;

  /**
   * @brief 用于上楼梯。
   * 将来将被删除。
   */
  float stairsVel;

  /**
   * @brief 控制频率，一般为 500Hz 或 1000Hz。
   */
  unsigned int controlFrequency = 500;

  /**
   * @brief 地面估计器中的滤波器窗口大小。
   */
  unsigned int filterWindowSize = 50;

  /**
   * @brief 速度估计器中的加速度计方差。用于卡尔曼滤波器。
   */
  float accelerometerVariance = 0.01f;

  /**
   * @brief 速度估计器中的传感器方差。用于卡尔曼滤波器。
   */
  float sensorVariance = 0.01f;

  /**
   * @brief 速度估计器中的初始方差。用于卡尔曼滤波器。
   */
  float initialVariance = 0.01f;

  /**
   * @brief 速度估计器中的移动窗口滤波器大小。
   */
  int movingWindowFilterSize = 50;

  /**
   * @brief.swing 控制器用于控制身体高度的期望值。
   */
  float desiredHeight = A1_BODY_HIGHT;

  /**
   * @brief 四足机器人的期望线速度。
   */
  Vec3<float> desiredSpeed = {0.f, 0.f, 0.f};

  /**
   * @brief Z 轴上的期望角速度。
   */
  float desiredTwistingSpeed = 0.f;

  /**
   * @brief 足部与地面的最大距离。
   */
  float footClearance = 0.01f;

  /**
   * @brief 摩擦系数。当前它们是常数。
   * 用于stance leg 控制器中。
   */
  Vec4<float> frictionCoeffs = {0.45f, 0.45f, 0.45f, 0.45f};

  /**
   * @brief Raibert 启发式的摆动 Kp。
   */
  std::map<std::string, std::vector<float>> swingKp;

  /**
   * @brief 是否在世界坐标系中计算力。
   */
  bool computeForceInWorldFrame = true;

  /**
   * @brief 是否在 MPC 计算后使用整体控制。
   */
  bool useWBC = true;
};

/**
 * @brief WbcCtrlData 类。
 * 该类存储所需状态。
 */
class qrWbcCtrlData {
public:
  /**
   * @brief 世界坐标系中的所需身体位置。
   */
  Vec3<float> pBody_des;

  /**
   * @brief 世界坐标系中的所需身体速度。
   */
  Vec3<float> vBody_des;

  /**
   * @brief 世界坐标系中的所需身体加速度。
   */
  Vec3<float> aBody_des;

  /**
   * @brief 所需身体 roll pitch yaw。
   */
  Vec3<float> pBody_RPY_des;

  /**
   * @brief 世界坐标系中的所需身体角速度。
   */
  Vec3<float> vBody_Ori_des;

  /**
   * @brief 世界坐标系中的所需 foothold 位置。
   */
  Vec3<float> pFoot_des[4];

  /**
   * @brief 世界坐标系中的所需 foothold 速度。
   */
  Vec3<float> vFoot_des[4];

  /**
   * @brief 世界坐标系中的所需 foothold 加速度。
   */
  Vec3<float> aFoot_des[4];

  /**
   * @brief 世界坐标系中的所需 foothold 力。
   */
  Vec3<float> Fr_des[4];

  /**
   * @brief 4 个 foothold 当前的接触状态。
   */
  Vec4<bool> contact_state;

  /**
   * @brief 是否进行 WBC。
   * 如果在一个迭代中同时进行 MPC 和 WBC，将消耗太多时间，
   * 所以如果在这个迭代中进行了 MPC，则将在下一个迭代中进行 WBC。
   */
  bool allowAfterMPC = true;
};

namespace Quadruped {

struct qrStateDataFlow {
  /**
   * @brief qrStateDataFlow 类的构造函数。
   */
  qrStateDataFlow();

  /**
   * @brief 基坐标系中的 foothold 位置。
   */
  Eigen::Matrix<float, 3, 4> footPositionsInBaseFrame;

  /**
   * @brief 基坐标系中的 foothold 速度。
   */
  Eigen::Matrix<float, 3, 4> footVelocitiesInBaseFrame;

  /**
   * @brief 世界坐标系中的基体线速度。
   */
  Vec3<float> baseVInWorldFrame;

  /**
   * @brief 世界坐标系中的基体角速度。
   */
  Vec3<float> baseWInWorldFrame;

  /**
   * @brief 基坐标系/IMU 帧中的基体线加速度。
   */
  Vec3<float> baseLinearAcceleration;

  /**
   * @brief 4 条腿的 foothold雅可比矩阵。
   */
  std::vector<Mat3<float>> footJvs;

  /**
   * @brief 基坐标系中的估计 foothold 力。
   */
  Eigen::Matrix<float, 3, 4> estimatedFootForce;

  /**
   * @brief 基坐标系中的估计力矩。
   */
  Vec3<float> estimatedMoment;

  /**
   * @brief 控制帧中的体高。
   */
  float heightInControlFrame = 0.27;

  /**
   * @brief 机器人的零动力点。
   */
  Vec3<float> zmp;

  /**
   * @brief 基坐标系到世界坐标系的旋转矩阵变换向量。
   */
  Mat3<float> baseRMat;

  /**
   * @brief 世界坐标系到控制帧的旋转矩阵变换向量。
   */
  Mat3<float> groundRMat;

  /**
   * @brief 基坐标系到控制帧的旋转矩阵变换向量。
   */
  Mat3<float> baseRInControlFrame;

  /**
   * @brief 控制帧的朝向。
   */
  Vec4<float> groundOrientation;

  /**
   * @brief 使用 Matplotlib 进行可视化。
   */
  Visualization2D visualizer;

  /**
   * @brief 保存用于整体控制的数据。
   */
  qrWbcCtrlData wbcData;
};

}  // namespace Quadruped

#endif  // QR_STATE_DATAFLOW_H
