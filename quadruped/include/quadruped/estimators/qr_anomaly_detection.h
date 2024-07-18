/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_CONTACT_DETECTION_H
#define QR_CONTACT_DETECTION_H

#include "quadruped/estimators/qr_moving_window_filter.hpp"
#include "quadruped/robots/qr_robot.h"
/* this is a external lib, but using some variables defined in qr_filter.hpp*/
#include "TinyEKF.h"

#include "quadruped/estimators/qr_ground_surface_estimator.h"
#include "quadruped/gait/qr_gait.h"
#include "quadruped/gait/qr_walk_gait_generator.h"
#include "quadruped/utils/physics_transform.h"

namespace Quadruped {

class qrContactDetection {
public:
  /**
   * @brief qrContactDetection 类的构造函数。
   * @param robotIn: 机器人类用于接触检测。
   * @param gaitGeneratorIn: 生成所需的步态计划以进行机动。
   * @param groundEstimatorIn: 估算脚部接触的 3D 平面。
   */
  qrContactDetection(qrRobot *robotIn, qrGaitGenerator *gaitGeneratorIn, qrGroundSurfaceEstimator *groundEstimatorIn);

  /**
   * @brief 更新接触状态，使用先前的数据和观测数据进行 Kalman 滤波估算。
   * 然后，计算是否发生滑动的概率。
   * @param currentTime: 自计时器启动以来的当前时间。
   */
  void Update(float currentTime);

  /**
   * @brief 重置估算。
   * @param currentTime: 自计时器启动以来的当前时间。
   */
  void Reset(float currentTime) {}

  /**
   * @brief 计算是否发生滑动的概率。
   * @param currentTime: 自计时器启动以来的当前时间。
   */
  void UpdateSlip(float currentTime);

  /**
   * @brief 警告：什么也不做。
   * @param currentTime: 自计时器启动以来的当前时间。
   */
  void GMObserver(float currentTime);

  /**
   * @brief 计算腿部的外部扭矩。
   * @param currentTime: 自计时器启动以来的当前时间。
   * @return 腿部的外部扭矩。
   */
  Vec4<float> JointObserver(float currentTime);

  /**
   * @brief 获取 isContact 成员的 getter 方法。
   */
  Vec4<bool> GetIsContact() { return isContact; };

private:
  /**
   * @brief 异常检测用的机器人类。
   */
  qrRobot *robot;

  /**
   * @brief 生成所需的步态计划以进行机动。
   */
  qrGaitGenerator *gaitGenerator;

  /**
   * @brief 估算脚部接触的 3D 平面。
   */
  qrGroundSurfaceEstimator *groundEstimator;

  /**
   * @brief 用于接触状态检测的 Kalman 滤波器。
   */
  TinyEKF<4, 12> *filterContact;  // dim state=4, dim obs=8

  /**
   * @brief 用于滑动检测的 Kalman 滤波器。
   */
  TinyEKF<4, 4> *filterSlip;  // dim state=4, dim obs=4

  /**
   * @brief 自计时器重新启动以来的时间。
   */
  float timeSinceReset;

  /**
   * @brief 存储最后的时间戳。
   */
  float lastTime;

  /**
   * @brief 存储控制框架中的最后一个 foothold 速度。
   */
  Vec4<float> lastFootVzInControlFrame = Vec4<float>::Zero();

  /**
   * @brief 机器人每条腿的接触结果。
   */
  Vec4<bool> isContact = {true, true, true, true};  // 机器人每条腿的接触结果

  /**
   * @brief 接触检测阈值。
   */
  double thresold[2] = {0.5, 0.4};

  /**
   * @brief 每个观测因子的接触概率。
   */
  Eigen::Matrix<float, 4, 4> pContact;  // 每个观测因子的接触概率

  /**
   * @brief 足部速度的移动窗口滤波器。
   */
  qrMovingWindowFilter<float, 3> windowFilter[4];

  // 卡尔曼滤波计算
  /**
   * @brief (卡尔曼滤波:) 状态转移函数。
   */
  double fx[4];  // fx = f(x,u)

  /**
   * @brief (卡尔曼滤波:) 状态量到观测量的转移函数。
   */
  double hx[12];  // hx = h(x)，由 H 矩阵确定。

  /**
   * @brief (卡尔曼滤波:) 状态转移矩阵。
   */
  double F[4][4];  // F = df/dx

  /**
   * @brief (卡尔曼滤波:) 观测矩阵。
   */
  double H[12][4];  // H = dh/dx

  /**
   * @brief (卡尔曼滤波:) 观测量。
   */
  double z[12];  // z = hx，真实观测量。

  // 卡尔曼滤波器参数
  /**
   * @brief 接触概率估算参数，
   * 这种方法在 MIT cheetah3：虚拟预测支持多边形中解释。
   */
  float sigmaPhase = 0.1f;

  /**
   * @brief 机器人腿部的外部扭矩。
   */
  Vec4<float> externalTorques;
  // for trot
  // Vec4<float> meanTorques = {4.0f,4.0f,4.0f,4.0f};
  // Vec4<float> sigmaTorques = {3.0f,3.0f,3.0f,3.0f};

  // for walk
  /**
   * @brief 4 条腿外部扭矩的均值。
   */
  Vec4<float> meanTorques = {1.0f, 1.0f, 1.0f, 1.0f};

  /**
   * 用于计算 EKF 观测器的方差。
   */
  Vec4<float> sigmaTorques = {2.0f, 2.0f, 2.0f, 2.0f};

  /**
   * 用于计算 EKF 观测器的方差。
   */
  Vec4<float> meanPz = {-A1_BODY_HIGHT, -A1_BODY_HIGHT, -A1_BODY_HIGHT, -A1_BODY_HIGHT};

  /**
   * 用于计算 EKF 观测器的方差。
   */
  Vec4<float> sigmaPz = {0.15f, 0.15f, 0.15f, 0.15f};

  // for leg force computation
  /**
   * @brief 腿部的外部力。
   */
  Eigen::Matrix<float, 3, 4> externalForces;

  /**
   * @brief 腿部惯性矩阵。
   */
  std::vector<Mat3<float>> legInertias;

  /**
   * @brief 上一个时间点的关节角速度。
   */
  Eigen::Matrix<float, 12, 1> lastJointVs;

  /// slip ///
  /**
   * @brief 每个观测因子的滑移概率。
   */
  Eigen::Matrix<float, 2, 4> pSlip;  // 每个观测因子的滑移概率

  /**
   * @brief 每条腿的滑移结果。
   */
  Vec4<bool> isSlip = {false, false, false, false};  // 每条腿的滑移结果

  /**
   * @brief 存储一段时间内的速度值，
   * 用于计算标准差。
   */
  std::deque<Vec3<float>> vQue;

  /**
   * @brief 一段时间内的速度值之和。
   */
  Vec3<float> vSum = Vec3<float>::Zero();

  /**
   * @brief 一段时间内的速度值的平均值。
   */
  Vec3<float> vAvg = Vec3<float>::Zero();

  /**
   * @brief (卡尔曼滤波器:) 滑移检测的状态转移函数。
   */
  double fxSlip[4];  // fx = f(x,u)

  /**
   * @brief (卡尔曼滤波器:) 状态量到观测量的转移函数。
   */
  double hxSlip[4];  // hx = h(x)，由 H 矩阵确定。

  /**
   * @brief (卡尔曼滤波器:) 状态转移矩阵。
   */
  double FSlip[4][4];  // F = df/dx

  /**
   * @brief (卡尔曼滤波器:) 观测矩阵。
   */
  double HSlip[4][4];  // H = dh/dx

  /**
   * @brief (卡尔曼滤波器:) 观测量。
   */
  double zSlip[4];  // z = hx，真实观测量。

  //! maybe need delete.
  long long count = 0;
};

/**
 * @brief 检测机器人脚部是否滑移
 */
class SlipDetection {
public:
  /**
   * @brief SlipDetection 类的构造函数
   * @param robotIn: 机器人类用于滑移检测
   * @param gaitGeneratorIn: 生成所需的步态计划用于机器人 locomotion
   * @param groundEstimatorIn: 估算脚部接触的 3D 平面
   */
  SlipDetection(qrRobot *robotIn, qrGaitGenerator *gaitGeneratorIn, qrGroundSurfaceEstimator *groundEstimatorIn);

  /**
   * @brief 检测机器人脚部是否滑移
   * @param currentTime: 自计时器启动以来的当前时间
   */
  //! maybe need delete
  void Update(float currentTime);

private:
  /**
   * @brief 滑移检测的机器人类
   */
  qrRobot *robot;

  /**
   * @brief 自计时器重启以来的时间
   */
  float timeSinceReset;

  /**
   * @brief 存储最后的时间戳
   */
  float lastTime;

  /**
   * @brief 机器人每条腿的接触结果
   */
  bool isContact;
};

/**
 * @brief 检测机器人腿部是否超出工作空间
 */
//! maybe need delete
class WorkspaceDetection {
public:
  /**
   * @brief 工作空间检测类的构造函数
   * @param robotIn: 机器人类用于接触检测
   * @param groundEstimatorIn: 估算脚部接触的 3D 平面
   */
  WorkspaceDetection(qrRobot *robotIn, qrGroundSurfaceEstimator *groundEstimatorIn)
      : robot(robotIn), groundEstimator(groundEstimator){};

  /**
   * @brief 使用 Cohen-Sutherland 方法检测脚部位置是否超出工作空间
   * 如果超出，则将脚部位置标准化到髋部坐标系中并限制在工作空间内
   * @return 限制后的脚部位置在基坐标系中
   */
  //! maybe need delete
  Eigen::Matrix<float, 3, 4> Update();

  //! maybe need delete
  void Detect() {}

private:
  float currentTime;

  Vec3<float> allowedWorkSpace = {0.1f, 0.1f, 0.05f};

  qrRobot *robot;

  qrGroundSurfaceEstimator *groundEstimator;
};

//! maybe need delete
class AnomalyDetection {
public:
  AnomalyDetection(qrRobot *robotIn, qrGaitGenerator *gaitGeneratorIn, qrGroundSurfaceEstimator *groundEstimatorIn);

private:
  qrRobot *robot;
  qrGaitGenerator *gaitGenerator;
  qrGroundSurfaceEstimator *groundEstimator;
  qrContactDetection contactDetection;
  SlipDetection slipDetection;
};

}  // Namespace Quadruped

#endif  // QR_CONTACT_DETECTION_H
