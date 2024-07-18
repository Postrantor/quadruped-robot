/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_POSE_PLANNER_H
#define QR_POSE_PLANNER_H

#include <map>
#include <set>

#include "estimators/qr_state_estimator_container.h"
#include "gait/qr_openloop_gait_generator.h"
#include "robots/qr_robot.h"
#include "utils/qr_geometry.h"
#include "utils/qr_se3.h"

namespace Quadruped {

/**
 * @brief 计划基准位置和基准姿态。注意，基准点和COM点不是同一个点。
 */
class qrPosePlanner {
public:
  /**
   * @brief footStepper 的重置时间。
   */
  float resetTime;

  /**
   * @brief 每条腿的id，在逆时针顺序。
   * 腿id：
   * 3---0
   * |   |
   * 1---2
   */
  const int legIdMap2CounterwiseOrder[4] = {0, 3, 1, 2};

  /**
   * @brief 每条腿的id，在顺时针顺序。
   * 腿id：
   * 1---0
   * |   |
   * 3---2
   */
  const int invLegIdMap2CounterwiseOrder[4] = {0, 2, 3, 1};

  /**
   * @brief qrRobot 对象。
   */
  qrRobot* robot;

  /**
   * @brief RobotEstimator 对象。
   */
  qrRobotEstimator* robotEstimator;

  /**
   * @brief GroundSurfaceEstimator 对象。
   */
  qrGroundSurfaceEstimator* groundEstimator;

  /**
   * @brief GaitGenerator 对象。
   */
  qrGaitGenerator* gaitGenerator;

  /**
   * @brief 髋部（偏移）位置在基准框架中。
   * 注意：见 [ETH:qrRobot Dynamics Lecture Notes] 了解这些表示。
   */
  Eigen::Matrix<float, 3, 4> rBH;

  /**
   * @brief 基准中心位置在世界/惯性框架中。
   */
  Vec3<float> rIB;

  /**
   * @brief 将 rIB 投影到世界XY平面。
   */
  Vec3<float> rIB_;

  /**
   * @brief 足部位置在基准框架中。
   */
  Eigen::Matrix<float, 3, 4> rBF;

  /**
   * @brief 足部位置在世界框架中。
   */
  Eigen::Matrix<float, 3, 4> rIF;

  /**
   * @brief COM 位置在基准框架中。
   */
  Vec3<float> rBCOM;

  /**
   * @brief COM 偏移位置在基准框架中。
   */
  Vec3<float> rICOMoffset;

  /**
   * @brief 投影的 COM 偏移位置在基准框架中。
   */
  Vec3<float> rICOMoffset_;

  /**
   * @brief 支撑多面体中心在世界框架中，包括 z 轴。
   */
  Vec3<float> rSP;

  /**
   * @brief 将 rSP 投影到 xy 平面。
   */
  Vec3<float> rSP_;

  /**
   * @brief 可能接触的腿的数量。
   */
  int N = 4;

  /**
   * @brief 有效的接触点腿的 Id。
   */
  std::set<int> validContactPointId;

  /**
   * @brief 允许的最小腿长。
   */
  float lMin = 0.22;

  /**
   * @brief 允许的最大腿长。
   */
  float lMax = 0.35;

  /**
   * @brief 用于梯度计算。
   * Asp 的大小可能是 (3,3) 或 (4, 3)
   */
  Eigen::MatrixXf Asp;

  /**
   * @brief 用于梯度计算。
   */
  Eigen::MatrixXf bsp;

  /**
   * @brief 用于 QP 解算。
   */
  Eigen::MatrixXf G;

  /**
   * @brief 拉格朗日因子。
   */
  Eigen::MatrixXf Lambda;

  /**
   * @brief 足部位置在基准框架中。
   */
  Eigen::Matrix<float, 3, 4> footPosition;

  /**
   * @brief 足部是否与地面接触。
   */
  bool contactK[4];

  /**
   * @brief 接触腿的数量。
   */
  float contactLegNumber;

  /**
   * @brief 是否是摆动脚。
   */
  bool swingK[4];

  /**
   * @brief 支撑多边形的顶点在世界框架中。
   */
  std::vector<Vec3<float>> supportPolygonVertices;

  /**
   * @brief 投影的支撑多边形的顶点在世界框架中。
   */
  std::vector<Vec3<float>> projectedSupportPolygonVertices;

  /**
   * @brief 足尖到基准的数组长度。
   */
  std::vector<Vec3<float>> g;

  /**
   * @brief 用于支撑多边形计算。
   */
  Vec3<float> so3Phi;

  /**
   * @brief 表示基准方向的四元数。
   */
  Quat<float> quat;

  /**
   * @brief 基准的 roll-pitch-yaw 在世界框架中。
   */
  Vec3<float> rpy;

  /**
   * @brief 估算的基准位置在世界框架中。
   */
  Vec3<float> rIBSource;

  /**
   * @brief 期望的基准姿态和位置。
   */
  Vec6<float> poseDest;

  /**
   * @brief 估算的基准姿态和位置在世界框架中。
   */
  Vec6<float> poseSource;

  /**
   * @brief 基准姿态和位置在世界框架中。
   */
  Vec6<float> pose;

  /**
   * @brief 基准的线速度和角速度在世界框架中。
   */
  Vec6<float> twist;

  /**
   * @brief 存储给定的数据。
   */
  robotics::math::qrSegment<float, Vec6<float>> segment;

  /**
   * @brief 用于梯度计算。
   */
  float omega = 0.5;

  /**
   * @brief 用于 G 计算。
   */
  float eps = 0.1;

  /**
   * @brief 初始 A1 机器人身体高度。
   */
  const float bodyHight = A1_BODY_HIGHT;

public:
  /**
   * @brief PosePlanner 类的构造函数。
   * @param robotIn:PosePlanner 所需的机器人。
   * @param gaitGeneratorIn: 生成期望的步态。
   * @param stateEstimators: 估算当前运动状态。
   */
  qrPosePlanner(qrRobot* robotIn, qrGaitGenerator* gaitGeneratorIn, qrStateEstimatorContainer* stateEstimators);

  /**
   * @brief PosePlanner 类的析构函数。
   */
  ~qrPosePlanner() = default;

  /**
   * @brief 在控制器开始时调用。
   * @param current_time:墙面时间，以秒为单位。
   */
  void Reset(float currentTime){};

  /**
   * @brief poseDest 成员的 getter 方法。
   */
  inline Vec6<float> GetBasePose() { return poseDest; };

  /**
   * @brief 重置 BasePose 规划器。
   * @param currentTime:墙面时间，以秒为单位。
   */
  void ResetBasePose(float currentTime) {
    resetTime = currentTime;

    Eigen::Matrix<float, 3, 4> footPoseWorld = robot->GetFootPositionsInWorldFrame();
    pose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw();  // poseDest;
    poseDest << footPoseWorld.row(0).mean(), footPoseWorld.row(1).mean(), bodyHight, 0, 0, 0;
    twist << 0, 0, 0, 0, 0, 0;
    std::cout << "resetTime = " << resetTime << ", reset poseDest = " << poseDest.transpose() << std::endl;
    segment.Reset(pose, poseDest);
  };

  /**
   * @brief 给定当前时间，获取相应的位置和姿态。
   * @param currentTime：墙面时间，以秒为单位。
   * @return 相应的位置和姿态。
   */
  std::tuple<Vec6<float>, Vec6<float>> GetIntermediateBasePose(float currentTime) {
    float dt = currentTime - resetTime;
    const float auxTime = 5.0;
    float phase = dt / auxTime;
    if (phase > 1.0) {
      phase = 1.0;
    }
    pose = segment.GetPoint(phase);
    std::cout << "currentTime" << currentTime << ", dt = " << dt << " phase=" << phase << std::endl;
    return {pose, twist};
  };

  /**
   * @brief 给定相对时间，获取相应的位置和姿态。
   * @param phase：相对时间在步态周期中的位置。
   * @param currentTime：墙面时间，以秒为单位。
   * @return 相应的位置和姿态。
   */
  std::tuple<Vec6<float>, Vec6<float>> GetIntermediateBasePose(float phase, float currentTime) {
    phase *= 1.0;
    if (phase > 1.0) {
      phase = 1.0;  // todo
    }
    pose = segment.GetPoint(phase);
    // pose[2] = poseDest[2];

    // robotics::math::Spline::Point point;
    // for(int i=0;i<3;++i) {
    //     tarjectoryR3[i].getPoint(currentTime, point);// segment.GetPoint(phase).head(3);
    //     pose[i] = point.x;
    //     twist[i] = point.xd;
    // }
    // Mat3<float> outR;
    // Vec3<float> outwb;
    // tarjectorySo3.GetPoint(currentTime, outR, outwb);
    // pose.tail(3) = robotics::math::rotationMatrixToRPY(outR.transpose());
    // twist.tail(3) = outwb;
    return {pose, twist};
  };

  /**
   * @brief 计算期望姿态目标。
   * @param currentTime：墙面时间，以秒为单位。
   * @return 期望姿态目标。
   */
  Vec6<float> Update(float currentTime);

  /**
   * @brief 计算 F 的梯度用于QP。
   * @return F 的梯度。
   */
  Vec6<float> ComputeGradientF();

  /**
   * @brief 计算 F 的 Hessian 矩阵用于QP。
   * @return F 的 Hessian 矩阵。
   */
  Mat6<float> ComputeHessianF();

  /**
   * @brief 计算 G 的梯度用于QP。
   * @return G 的梯度。
   */
  Eigen::MatrixXf ComputeGradientG();

  /**
   * @brief 计算 G 的 Hessian 矩阵用于QP。
   * @return G 的 Hessian 矩阵。
   */
  std::vector<Mat6<float>> ComputeHessianG();

  /**
   * @brief 计算 G。
   * @return G。
   */
  Eigen::MatrixXf ComputeG();

  /**
   * @brief 计算 F。
   * @return F。
   */
  float ComputeF();

  /**
   * @brief  projection 向量。
   * @param 输入向量。
   * @return 投影后的向量。
   */
  Vec3<float> ProjectV(Vec3<float> v);

  /**
   * @brief 解决 qp 问题。
   * @param hessF。
   * @param hessGSum。
   * @param gradientF。
   * @param gradientG。
   * @param GValue。
   * @return 期望姿态和拉格朗日因子。
   */
  std::tuple<Vec6<float>, Eigen::MatrixXf> QpSolver(
      Mat6<float>& hessF,
      Mat6<float>& hessGSum,
      Vec6<float>& gradientF,
      Eigen::MatrixXf& gradientG,
      Eigen::MatrixXf& GValue);

  /**
   * @brief 将矩阵列进行逆时针旋转。
   * @param 输入&输出矩阵。
   */
  void ToCounterClockOrder(Eigen::Matrix<float, 3, 4>& A) {
    Eigen::PermutationMatrix<4, 4> perm;
    perm.indices() = {0, 2, 3, 1};
    // Permutate cols
    A = A * perm;
  };

  /**
   * @brief 将数组列进行逆时针旋转。
   * @param 输入&输出数组。
   */
  void ToCounterClockOrder(bool array[4]) {
    bool temp = array[1];
    array[1] = array[2];
    array[2] = array[3];
    array[3] = temp;
  };
};

}  // Namespace Quadruped

#endif  // QR_POSE_PLANNER_H
