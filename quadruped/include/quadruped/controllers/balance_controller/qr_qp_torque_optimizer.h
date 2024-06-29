/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_QP_TORQUE_OPTIMIZER_H
#define QR_QP_TORQUE_OPTIMIZER_H

#include <Eigen/Dense>
#include <tuple>

#include "estimators/qr_ground_surface_estimator.h"
#include "robots/qr_robot.h"

namespace Quadruped {

/**
 * @brief 在控制框架中计算力平衡问题的逆质量矩阵。
 * @attention 输入应该在控制框架中表示。
 * @param robotMass: 四足机器人总质量。
 * @param robotInertia: 四足机器人整体惯性矩阵在控制框架中。
 * 这可以被视为虚拟惯性。
 * @param footPositions: 4个 foothold 在控制框架中的位置。
 * @return 逆质量矩阵。
 */
Eigen::Matrix<float, 6, 12> ComputeMassMatrix(
    float robotMass, Eigen::Matrix<float, 3, 3> robotInertia, Eigen::Matrix<float, 4, 3> footPositions);

/**
 * @brief 在世界框架中计算力平衡问题的逆质量矩阵。
 * @param robotMass: 四足机器人总质量。
 * @param robotInertia: 四足机器人整体惯性矩阵在控制框架中。
 * 这可以被视为虚拟惯性。
 * @param footPositions: 4个 foothold 在控制框架中的位置。
 * @param rotMat: 从基框架到世界框架的旋转矩阵。
 * @return 逆质量矩阵。
 */
Eigen::Matrix<float, 6, 12> ComputeMassMatrix(
    float robotMass,
    Eigen::Matrix<float, 3, 3> robotInertia,
    Eigen::Matrix<float, 4, 3> footPositions,
    Mat3<float> rotMat);

/**
 * @brief 计算二次规划问题的约束矩阵，包括摩擦圆锥和力限制。
 * @param bodyMass: 机器人总质量。
 * @param contacts: 4个 foothold 与地面接触的状态。
 * @param frictionCoef: 定义脚部与环境之间的交互力效应。
 * @param fMinRatio: 最小应用力比率。
 * @param fMaxRatio: 最大应用力比率。
 * @param surfaceNormal: 地形表面的法向量。在这个函数中固定值。
 * 如果用户想要调整调试，可以更改这个值。
 * @return 约束矩阵。
 */
std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> ComputeConstraintMatrix(
    float bodyMass,
    Eigen::Matrix<bool, 4, 1> contacts,
    float frictionCoef,
    float fMinRatio,
    float fMaxRatio,
    Vec3<float> surfaceNormal = {0.f, 0.f, 1.f});

/**
 * @brief 计算约束矩阵。
 * @param bodyMass: 机器人总质量。
 * @param contacts: 4个 foothold 与地面接触的状态。
 * @param frictionCoef: 定义脚部与环境之间的交互力效应。
 * @param fMinRatio: 4个元素的最小应用力向量。
 * @param fMaxRatio: 4个元素的最大应用力向量。
 * @param normal: 地形表面的法向量。
 * @param tangent1: X轴上的角度。用于计算摩擦圆锥。
 * @param tangent2: Y轴上的角度。用于计算摩擦圆锥。
 * @return 约束矩阵。
 */
std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> ComputeConstraintMatrix(
    float bodyMass,
    Eigen::Matrix<bool, 4, 1> contacts,
    float frictionCoef,
    Vec4<float> fMinRatio,
    Vec4<float> fMaxRatio,
    Vec3<float> normal,
    Vec3<float> tangent1,
    Vec3<float> tangent2);

/**
 * @brief 计算二次规划问题的目标矩阵。
 * @param massMatrix: 之前计算的质量矩阵。
 * @param desiredAcc: 由KP/KD控制计算的期望加速度向量。
 * @param accWeight: 6个加速度分量的权重。
 * @param regWeight: 保持正定的小值。
 * @param g: 重力加速度向量。
 * @return 目标矩阵。
 */
std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> ComputeObjectiveMatrix(
    Eigen::Matrix<float, 6, 12> massMatrix,
    Eigen::Matrix<float, 6, 1> desiredAcc,
    Eigen::Matrix<float, 6, 1> accWeight,
    float regWeight,
    Eigen::Matrix<float, 6, 1> g);

/**
 * @brief 计算调整权重。
 * @param robot: 指向Robot类的指针。
 * 这可能在未来被删除。
 * @param contacts: 4个 foothold 与地面接触的状态。
 * @return 调整权重矩阵。
 */
Eigen::Matrix<float, 12, 12> ComputeWeightMatrix(qrRobot *robot, const Eigen::Matrix<bool, 4, 1> &contacts);

/**
 * @brief 通过力平衡方法计算期望接触力。
 * 力是以基框架表示的。
 * @param robot: 指向Robot类的指针。
 * @param groundEstimator: 指向GroundEstimator类的指针。
 * @param desiredAcc: 由KP/KD控制计算的期望加速度向量。
 * @param contacts: 4个 foothold 与地面接触的状态。
 * @param accWeight: 6个加速度分量的权重。
 * @param regWeight: 保持正定的小值。
 * @param frictionCoef: 定义脚部与环境之间的交互力效应。
 * @param fMinRatio: 最小应用力比率。
 * @param fMaxRatio: 最大应用力比率。
 * @return 应用到每个 foothold 的力矩阵。
 */
Eigen::Matrix<float, 3, 4> ComputeContactForce(
    qrRobot *robot,
    qrGroundSurfaceEstimator *groundEstimator,
    Eigen::Matrix<float, 6, 1> desiredAcc,
    Eigen::Matrix<bool, 4, 1> contacts,
    Eigen::Matrix<float, 6, 1> accWeight,
    float regWeight = 1e-4,
    float frictionCoef = 0.5f,
    float fMinRatio = 0.01f,
    float fMaxRatio = 10.f);

/**
 * @brief 通过力平衡方法计算期望接触力。
 * 这个函数计算世界框架中的接触力。
 * @param robot: 指向Robot类的指针。
 * @param desiredAcc: 由KP/KD控制计算的期望加速度向量。
 * @param contacts: 4个 foothold 与地面接触的状态。
 * @param accWeight: 6个加速度分量的权重。
 * @param normal: 地形表面的法向量。
 * @param tangent1: X轴上的角度。用于计算摩擦圆锥。
 * @param tangent2: Y轴上的角度。用于计算摩擦圆锥。
 * @param fMinRatio: 最小应用力比率。用户可以调整每个腿部的参数。
 * @param fMaxRatio: 最大应用力比率。用户可以调整每个腿部的参数。
 * @param regWeight: 保持正定的小值。
 * @param frictionCoef: 定义脚部与环境之间的交互力效应。
 * @return 应用到每个 foothold 的力矩阵。
 */
Eigen::Matrix<float, 3, 4> ComputeContactForce(
    qrRobot *robot,
    Eigen::Matrix<float, 6, 1> desiredAcc,
    Eigen::Matrix<bool, 4, 1> contacts,
    Eigen::Matrix<float, 6, 1> accWeight,
    Vec3<float> normal,
    Vec3<float> tangent1,
    Vec3<float> tangent2,
    Vec4<float> fMinRatio = {0.01f, 0.01f, 0.01f, 0.01f},
    Vec4<float> fMaxRatio = {10.f, 10.f, 10.f, 10.f},
    float regWeight = 1e-4,
    float frictionCoef = 0.6f);

}  // namespace Quadruped

#endif  // QR_QP_TORQUE_OPTIMIZER_H
