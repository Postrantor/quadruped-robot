/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_SE3_H
#define QR_SE3_H

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <type_traits>

#include "quadruped/utils/qr_algebra.h"
#include "quadruped/utils/qr_cpptypes.h"

namespace robotics {

static constexpr double quaternionDerviativeStabilization = 0.1;

namespace math {

enum class CoordinateAxis { X, Y, Z };

/**
 * @brief 将弧度转换为角度。
 */
template <typename T>
T rad2deg(T rad) {
  static_assert(std::is_floating_point<T>::value, "must use floating point value");
  return rad * T(180) / T(M_PI);
}

/**
 * @brief 将角度转换为弧度。
 */
template <typename T>
T deg2rad(T deg) {
  static_assert(std::is_floating_point<T>::value, "must use floating point value");
  return deg * T(M_PI) / T(180);
}

/**
 * @brief 计算坐标变换的旋转矩阵。 请注意，
 * coordinateRotation(CoordinateAxis:X, .1) * v 将使 v 旋转 -.1 弧度 -
 * 这将转换为旋转 .1 弧度的坐标系！。
 */
template <typename T>
Mat3<T> coordinateRotation(CoordinateAxis axis, T theta) {
  static_assert(std::is_floating_point<T>::value, "must use floating point value");
  T s = std::sin(theta);
  T c = std::cos(theta);

  Mat3<T> R;

  if (axis == CoordinateAxis::X) {
    R << 1, 0, 0, 0, c, s, 0, -s, c;
  } else if (axis == CoordinateAxis::Y) {
    R << c, 0, -s, 0, 1, 0, s, 0, c;
  } else if (axis == CoordinateAxis::Z) {
    R << c, s, 0, -s, c, 0, 0, 0, 1;
  }

  return R;
}

/**
 * @brief 将向量转换为叉积矩阵。
 */
template <typename T>
Mat3<typename T::Scalar> crossMatrix(const Eigen::MatrixBase<T> &v) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3, "must have 3x1 vector");
  Mat3<typename T::Scalar> m;
  m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return m;
}

/**
 * @brief 将欧拉角转换为旋转矩阵。
 */
template <typename T>
Mat3<typename T::Scalar> rpyToRotMat(const Eigen::MatrixBase<T> &v) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3, "must have 3x1 vector");
  Mat3<typename T::Scalar> m = coordinateRotation(CoordinateAxis::X, v[0]) *
                               coordinateRotation(CoordinateAxis::Y, v[1]) *
                               coordinateRotation(CoordinateAxis::Z, v[2]);
  return m;
}

/**
 * @brief 将向量转换为斜对称矩阵。
 */
template <typename T>
Mat3<typename T::Scalar> vectorToSkewMat(const Eigen::MatrixBase<T> &v) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3, "Must have 3x1 matrix");
  Mat3<typename T::Scalar> m;
  m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return m;
}

/**
 * @brief 将斜对称矩阵转换为向量。
 */
template <typename T>
Vec3<typename T::Scalar> matToSkewVec(const Eigen::MatrixBase<T> &m) {
  static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3, "Must have 3x3 matrix");
  return 0.5 * Vec3<typename T::Scalar>(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0), (m(1, 0) - m(0, 1)));
}

/**
 * @brief 将坐标变换矩阵转换为方向四元数。
 */
template <typename T>
Quat<typename T::Scalar> rotationMatrixToQuaternion(const Eigen::MatrixBase<T> &r1) {
  static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3, "Must have 3x3 matrix");
  Quat<typename T::Scalar> q;
  Mat3<typename T::Scalar> r = r1.transpose();
  typename T::Scalar tr = r.trace();
  if (tr > 0.0) {
    typename T::Scalar S = sqrt(tr + 1.0) * 2.0;
    q(0) = 0.25 * S;
    q(1) = (r(2, 1) - r(1, 2)) / S;
    q(2) = (r(0, 2) - r(2, 0)) / S;
    q(3) = (r(1, 0) - r(0, 1)) / S;
  } else if ((r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2))) {
    typename T::Scalar S = sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
    q(0) = (r(2, 1) - r(1, 2)) / S;
    q(1) = 0.25 * S;
    q(2) = (r(0, 1) + r(1, 0)) / S;
    q(3) = (r(0, 2) + r(2, 0)) / S;
  } else if (r(1, 1) > r(2, 2)) {
    typename T::Scalar S = sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
    q(0) = (r(0, 2) - r(2, 0)) / S;
    q(1) = (r(0, 1) + r(1, 0)) / S;
    q(2) = 0.25 * S;
    q(3) = (r(1, 2) + r(2, 1)) / S;
  } else {
    typename T::Scalar S = sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
    q(0) = (r(1, 0) - r(0, 1)) / S;
    q(1) = (r(0, 2) + r(2, 0)) / S;
    q(2) = (r(1, 2) + r(2, 1)) / S;
    q(3) = 0.25 * S;
  }
  return q;
}

/**
 * @brief 将四元数转换为旋转矩阵。 该矩阵表示坐标变换到由四元数指定的姿态的坐标系。
 */
template <typename T>
Mat3<typename T::Scalar> quaternionToRotationMatrix(const Eigen::MatrixBase<T> &q) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4, "Must have 4x1 quat");
  typename T::Scalar e0 = q(0);
  typename T::Scalar e1 = q(1);
  typename T::Scalar e2 = q(2);
  typename T::Scalar e3 = q(3);

  Mat3<typename T::Scalar> R;

  R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3), 2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
      1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1), 2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
      1 - 2 * (e1 * e1 + e2 * e2);
  R.transposeInPlace();
  return R;
}

/**
 * @brief 将四元数转换为欧拉角。 使用 ZYX 顺序（偏航-俯仰-滚转），但返回
 * 的角度顺序为（滚转、俯仰、偏航）。
 */
template <typename T>
Vec3<typename T::Scalar> quatToRPY(const Eigen::MatrixBase<T> &q) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4, "Must have 4x1 quat");
  Vec3<typename T::Scalar> rpy;
  typename T::Scalar as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
  rpy(2) = std::atan2(2 * (q[1] * q[2] + q[0] * q[3]), square(q[0]) + square(q[1]) - square(q[2]) - square(q[3]));
  rpy(1) = std::asin(as);
  rpy(0) = std::atan2(2 * (q[2] * q[3] + q[0] * q[1]), square(q[0]) - square(q[1]) - square(q[2]) + square(q[3]));
  return rpy;
}

/**
 * @brief 将欧拉角转换为四元数。
 */
template <typename T>
Quat<typename T::Scalar> rpyToQuat(const Eigen::MatrixBase<T> &rpy) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3, "Must have 3x1 vec");
  Mat3<typename T::Scalar> R = rpyToRotMat(rpy);
  Quat<typename T::Scalar> q = rotationMatrixToQuaternion(R);
  return q;
}

/**
 * @brief 将四元数转换为 so3 向量。
 */
template <typename T>
Vec3<typename T::Scalar> quatToso3(const Eigen::MatrixBase<T> &q) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4, "Must have 4x1 quat");
  Vec3<typename T::Scalar> so3;
  typename T::Scalar theta = 2. * std::acos(q[0]);
  so3[0] = theta * q[1] / std::sin(theta / 2.);
  so3[1] = theta * q[2] / std::sin(theta / 2.);
  so3[2] = theta * q[3] / std::sin(theta / 2.);
  return so3;
}

/**
 * @brief 将旋转矩阵转换为欧拉角。
 */
template <typename T>
Vec3<typename T::Scalar> rotationMatrixToRPY(const Eigen::MatrixBase<T> &R) {
  static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3, "Must have 3x3 matrix");
  Quat<typename T::Scalar> q = rotationMatrixToQuaternion(R);
  Vec3<typename T::Scalar> rpy = quatToRPY(q);
  return rpy;
}

/**
 * @brief 四元数导数计算，类似于 MATLAB 中的 rqd(q, omega) 函数。
 * omega 表示在本体坐标系下的角速度。
 */
template <typename T, typename T2>
Quat<typename T::Scalar> quatDerivative(const Eigen::MatrixBase<T> &q, const Eigen::MatrixBase<T2> &omega) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4, "Must have 4x1 quat");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 omega");
  /* 对应 rqd 函数中的第一种情况。 */
  Mat4<typename T::Scalar> Q;
  Q << q[0], -q[1], -q[2], -q[3], q[1], q[0], -q[3], q[2], q[2], q[3], q[0], -q[1], q[3], -q[2], q[1], q[0];

  Quat<typename T::Scalar> qq(
      quaternionDerviativeStabilization * omega.norm() * (1 - q.norm()), omega[0], omega[1], omega[2]);
  Quat<typename T::Scalar> dq = 0.5 * Q * qq;
  return dq;
}

/**
 * @brief 计算两个四元数的积。
 */
template <typename T>
Quat<typename T::Scalar> quatProduct(const Eigen::MatrixBase<T> &q1, const Eigen::MatrixBase<T> &q2) {
  typename T::Scalar r1 = q1[0];
  typename T::Scalar r2 = q2[0];
  Vec3<typename T::Scalar> v1(q1[1], q1[2], q1[3]);
  Vec3<typename T::Scalar> v2(q2[1], q2[2], q2[3]);

  typename T::Scalar r = r1 * r2 - v1.dot(v2);
  Vec3<typename T::Scalar> v = r1 * v2 + r2 * v1 + v1.cross(v2);
  Quat<typename T::Scalar> q(r, v[0], v[1], v[2]);
  return q;
}

/**
 * @brief 获取四元数的逆。
 */
template <typename T>
Quat<typename T::Scalar> quatInverse(const Eigen::MatrixBase<T> &q) {
  Quat<typename T::Scalar> qInverse;
  qInverse << q[0], -q.tail(3);
  qInverse /= q.dot(q);
  return qInverse;
}

/**
 * @brief 计算新的四元数：
 * @param quat: 旧的四元数
 * @param omega: 角速度（在惯性坐标系下！）
 * @param dt: 时间步长
 * @return 新的四元数
 */
template <typename T, typename T2, typename T3>
Quat<typename T::Scalar> integrateQuat(const Eigen::MatrixBase<T> &quat, const Eigen::MatrixBase<T2> &omega, T3 dt) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4, "Must have 4x1 quat");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 omega");
  Vec3<typename T::Scalar> axis;
  typename T::Scalar ang = omega.norm();
  if (ang > 0) {
    axis = omega / ang;
  } else {
    axis = Vec3<typename T::Scalar>(1, 0, 0);
  }

  ang *= dt;
  Vec3<typename T::Scalar> ee = std::sin(ang / 2) * axis;
  Quat<typename T::Scalar> quatD(std::cos(ang / 2), ee[0], ee[1], ee[2]);

  Quat<typename T::Scalar> quatNew = quatProduct(quatD, quat);
  quatNew = quatNew / quatNew.norm();
  return quatNew;
}

/**
 * @brief 计算新的四元数：
 * @param quat: 旧的四元数
 * @param omega: 角速度（在惯性坐标系下！）
 * @param dt: 时间步长
 * @return 新的四元数
 */
template <typename T, typename T2, typename T3>
Quat<typename T::Scalar> integrateQuatImplicit(
    const Eigen::MatrixBase<T> &quat, const Eigen::MatrixBase<T2> &omega, T3 dt) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4, "Must have 4x1 quat");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 omega");
  Vec3<typename T::Scalar> axis;
  typename T::Scalar ang = omega.norm();
  if (ang > 0) {
    axis = omega / ang;
  } else {
    axis = Vec3<typename T::Scalar>(1, 0, 0);
  }

  ang *= dt;
  Vec3<typename T::Scalar> ee = std::sin(ang / 2) * axis;
  Quat<typename T::Scalar> quatD(std::cos(ang / 2), ee[0], ee[1], ee[2]);

  Quat<typename T::Scalar> quatNew = quatProduct(quat, quatD);
  quatNew = quatNew / quatNew.norm();
  return quatNew;
}

/**
 * @brief 将四元数转换为 so3 向量。
 */
template <typename T>
void quaternionToso3(const Quat<T> quat, Vec3<T> &so3) {
  so3[0] = quat[1];
  so3[1] = quat[2];
  so3[2] = quat[3];

  T theta = 2.0 * asin(sqrt(so3[0] * so3[0] + so3[1] * so3[1] + so3[2] * so3[2]));

  if (fabs(theta) < 0.0000001) {
    so3.setZero();
    return;
  }
  so3 /= sin(theta / 2.0);
  so3 *= theta;
}

/**
 * @brief 将 so3 转换为四元数。
 */
template <typename T>
Quat<T> so3ToQuat(Vec3<T> &so3) {
  Quat<T> quat;

  T theta = sqrt(so3[0] * so3[0] + so3[1] * so3[1] + so3[2] * so3[2]);

  if (fabs(theta) < 1.e-6) {
    quat.setZero();
    quat[0] = 1.;
    return quat;
  }
  quat[0] = cos(theta / 2.);
  quat[1] = so3[0] / theta * sin(theta / 2.);
  quat[2] = so3[1] / theta * sin(theta / 2.);
  quat[3] = so3[2] / theta * sin(theta / 2.);
  return quat;
}

/**
 * @brief 计算逆刚体变换。
 * @param t: 基坐标系原点在世界坐标系中的3D位置;
 * @param R: 基坐标系的朝向（四元数，wxyz）相对于世界坐标系;
 * @return Mat4<T> 变换矩阵 invMat.
 */
template <typename T>
Mat4<T> invertTransform(Vec3<T> t, Quat<T> R) {
  Mat4<T> invMat = Mat4<T>::Identity();
  auto RInverse = quaternionToRotationMatrix(R);  // .transpose();
  invMat.block(0, 0, 3, 3) = RInverse;
  invMat.block(0, 3, 3, 1) = -RInverse * t;
  return invMat;
}

/**
 * @brief 将基坐标系中的点坐标转换到世界坐标系中。
 * @param t: 基坐标系原点在世界坐标系中的3D位置;
 * @param R: 基坐标系的朝向（四元数，wxyz）相对于世界坐标系;
 * @param points: 输入点在基坐标系中的表示;
 * @return Eigen::Matrix<T, 3, N> 输出点在世界坐标系中的表示，但它们是同一个物理点。
 */
template <typename T, int N = 1>
Eigen::Matrix<T, 3, N> invertRigidTransform(Vec3<T> t, Quat<T> R, Eigen::Matrix<T, 3, N> points) {
  Isometry3<T> transform = Isometry3<T>::Identity();
  transform.translate(t);
  Eigen::Quaternion<T> r{R(0), R(1), R(2), R(3)};
  transform.rotate(r);
  return transform * points;
}

/**
 * @brief 将世界坐标系中的点坐标转换到基坐标系中。
 * @param t: 基坐标系原点在世界坐标系中的3D位置;
 * @param R: 基坐标系的朝向（四元数，wxyz）相对于世界坐标系;
 * @param points: 输入点在世界坐标系中的表示;
 * @return Eigen::Matrix<T, 3, N> 输出点在基坐标系中的表示，但它们是同一个物理点。
 */
template <typename T, int N = 1>
Eigen::Matrix<T, 3, N> RigidTransform(Vec3<T> t, Quat<T> R, Eigen::Matrix<T, 3, N> points) {
  Isometry3<T> transform = Isometry3<T>::Identity();

  Eigen::Quaternion<T> r{R(0), R(1), R(2), R(3)};
  transform.rotate(r.inverse());
  transform.translate(-t);
  return transform * points;
}

/**
 * @brief 将矢量 r 在 B 坐标系中转换到 A 坐标系中。
 * @param quat: Quat<float> wxyz;
 * @param r_b: Vec3<float>,矢量/位置在 B 坐标系中的表示;
 */
template <typename T>
Vec3<T> TransformVecByQuat(Quat<T> quat, Vec3<T> r_b) {
  T q0 = quat[0];
  Vec3<T> q_ = quat.tail(3);
  Vec3<T> r_a = (2 * q0 * q0 - 1) * r_b + 2 * q0 * vectorToSkewMat(q_) * r_b + 2 * q_ * q_.dot(r_b);
  return r_a;
}

/**
 * @brief 两个四元数的连接。
 */
template <typename T>
Quat<T> ConcatenationTwoQuats(Quat<T> quat_CB, Quat<T> quat_BA) {
  T q0 = quat_CB[0];
  T p0 = quat_BA[0];

  Vec3<T> q_ = quat_CB.tail(3);
  Vec3<T> p_ = quat_BA.tail(3);
  Quat<T> res;
  res << q0 * p0 - q_.dot(p_), q0 * p_ + p0 * q_ + crossMatrix(q_) * p_;
  return res;
}

/**
 * @brief 四元数的对数。
 */
template <typename T>
Vec3<T> LogQuat(Quat<T> q) {
  T q0 = q[0];
  Vec3<T> q_ = q.tail(3);
  T q_norm = q_.norm();
  Vec3<T> so3 = 2 * atan2(q_.norm(), q0) * q_ / q_norm;
  return so3;
}

/**
 * @brief 将旋转矩阵转换为轴角。
 */
template <typename T>
Vec3<typename T::Scalar> rotationMatrixToAxisAngle(const Eigen::MatrixBase<T> &r) {
  static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3, "Must have 3x3 matrix");
  Mat3<typename T::Scalar> R = r.transpose();
  typename T::Scalar tr = R.trace();
  float theta = acos((tr - 1.0) / 2);
  Vec3<float> axis;
  axis << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
  if (abs(theta) < 0.1) {
    return axis * 0.5 / (1 - pow(theta, 2) / 6 + pow(theta, 4) / 120);
  } else {
    return axis * 0.5 / sin(theta) * theta;
  }
}

}  // Namespace math

}  // Namespace robotics

#endif  // QR_SE3_H
