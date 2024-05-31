/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_MPC_INTERFACE
#define QR_MPC_INTERFACE

#include <include/qpOASES.hpp>

#include "robots/qr_timer.h"
#include "utils/qr_se3.h"
#include "utils/qr_tools.h"

#define K_MAX_GAIT_SEGMENTS 16

namespace Quadruped {

class MPCRobotState {
public:
  /**
   * @brief 打印机器人状态。
   */
  void print();

  /**
   * @brief 世界坐标系中的位置。
   */
  Eigen::Matrix<float, 3, 1> p;

  /**
   * @brief 世界坐标系中的线速度。
   */
  Eigen::Matrix<float, 3, 1> v;

  /**
   * @brief 基坐标系中的角速度。
   */
  Eigen::Matrix<float, 3, 1> w;

  /**
   * @brief 基坐标系中的足端位置。
   */
  Eigen::Matrix<float, 3, 4> footPosInBaseFrame;

  /**
   * @brief 基坐标系到世界坐标系的旋转矩阵。
   */
  Mat3<float> rotMat;

  /**
   * @brief 只考虑航向角的旋转矩阵。
   */
  Mat3<float> yawRotMat;

  /**
   * @brief 基坐标系中的惯性矩阵。
   */
  Mat3<float> bodyInertia;

  /**
   * @brief 世界坐标系中的四元数。
   */
  Eigen::Quaternionf quat;

  /**
   * @brief 机器人的滚转、俯仰、偏航角。
   */
  Eigen::Matrix<float, 3, 1> rpy;

  /**
   * @brief 机器人的质量。
   */
  float mass = 12;

  /**
   * @brief 预测的机器人轨迹。最多允许 16 个未来时间段。
   */
  float traj[12 * K_MAX_GAIT_SEGMENTS];

  /**
   * @brief 每条腿的步态状态。每个值都是 STANCE 或 SWING。
   */
  float gait[4 * K_MAX_GAIT_SEGMENTS];
};

struct ProblemConfig {
  /**
   * @brief MPC 的一个步骤所需时间。
   */
  float dt;

  /**
   * @brief 足部和环境之间的交互力效应。
   */
  float frictionCoeff;

  /**
   * @brief 每条腿可以施加的最大力。
   */
  float fMax;

  /**
   * @brief MPC 预测horizon 的长度。
   */
  int horizon;

  /**
   * @brief 四足机器人的总质量。
   */
  float totalMass;

  /**
   * @brief 四足机器人的 12 个状态权重，包括姿态和扭矩。
   */
  float weights[12];

  /**
   * @brief 力的权重。此项用于 quadratic 问题中最小化力。
   * 也用于在状态范数和力范数之间进行权衡。
   */
  float alpha;
};

/**
 * @brief 设置 MPC 参数。该函数将填充 cpp 中的静态变量 %problemConfig。
 * @param dt：一个 MPC 步骤所需的时间。
 * @param horizon：MPC 考虑的未来步骤数。
 * @param frictionCoeff：定义足部和环境之间的交互力效应。
 * @param fMax：每条腿施加的最大力。
 * @param totalMass：四足机器人的总质量。
 * @param inertia：四足机器人在基坐标系中的惯性矩阵。
 * @param weight：12 元素权重向量，用于姿态和扭矩。
 * @param alpha：力在 QP 形式中的权重。
 */
void SetupProblem(
    double dt,
    int horizon,
    double frictionCoeff,
    double fMax,
    double totalMass,
    float *inertia,
    float *weight,
    float alpha);

/**
 * @brief 在构建 MPC 问题前重置静态矩阵。
 * @param horizon：MPC 考虑的步骤数。
 */
void ResizeQPMats(s16 horizon);

/**
 * @brief 将问题转换为离散时间动力学。
 * @param Ac：连续时间状态矩阵。
 * @param Bc：连续时间转换矩阵。
 * @param dt：一个 MPC 步骤所需的时间。
 * @param horizon：MPC 考虑的步骤数。
 */
void ConvertToDiscreteQP(Eigen::Matrix<float, 13, 13> Ac, Eigen::Matrix<float, 13, 12> Bc, float dt, s16 horizon);

/**
 * @brief [in] 连续时间状态空间矩阵，包括 A 和 B。
 * @param [in] I_world：世界坐标系中的惯性矩阵。
 * @param [in] mass：四足机器人的总质量。
 * @param [in] r_feet：足端位置到重心的矩阵。
 * @param [in] yawRotMat：只包含航向角的旋转矩阵。
 * @param [out] A：输出，状态向量的乘法矩阵。
 * @param [out] B：输出，输入向量的乘法矩阵。
 */
void ComputeContinuousTimeStateSpaceMatrices(
    Mat3<float> I_world,
    float mass,
    Eigen::Matrix<float, 3, 4> r_feet,
    Mat3<float> yawRotMat,
    Eigen::Matrix<float, 13, 13> &A,
    Eigen::Matrix<float, 13, 12> &B);

/**
 * @brief 解决 MPC 问题。
 * 这个函数实际上构建了 QP 形式并使用 qpOASES 解决它。
 * @param p：四足机器人在世界坐标系中的位置。
 * @param v：四足机器人在世界坐标系中的速度。
 * @param q：四元数表示的旋转。
 * @param w：四足机器人的角速度。
 * @param r：4 个足端位置到重心的矩阵。
 * @param rpy：四足机器人的滚转、俯仰、偏航角。
 * @param state_trajectory：之前生成的未来状态轨迹。
 * @param gait： horzion 步骤中的步态状态。通常是 STANCE 或 SWING。
 */
void SolveMPCKernel(
    Vec3<float> &p,
    Vec3<float> &v,
    Quat<float> &q,
    Vec3<float> &w,
    Eigen::Matrix<float, 3, 4> &r,
    Vec3<float> &rpy,
    float *state_trajectory,
    float *gait);

/**
 * @brief 解决 MPC 问题。准备必要的数据以供 MPC 使用，并调用 %SolveMPCKernel 解决它。
 * @param setup：MPC 问题的一些参数。
 */
void SolveMPC(ProblemConfig *setup);

/**
 * @brief 获取 MPC 解的值，该值以 qpOASES 浮点向量形式出现。
 * @param index：结果的索引。
 * @return MPC 解的结果。
 */
double GetMPCSolution(int index);

}  // Namespace Quadruped

#endif  // QR_MPC_INTERFACE
