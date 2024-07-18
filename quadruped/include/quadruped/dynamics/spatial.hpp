/*! @file spatial.h
 * @brief 空间量操作实用函数
 * 该文件包含了空间向量和变换矩阵的操作函数。
 */

#ifndef LIBBIOMIMETICS_SPATIAL_H
#define LIBBIOMIMETICS_SPATIAL_H

#include <iostream>
#include <type_traits>

#include "quadruped/utils/qr_se3.h"

namespace spatial {
using namespace robotics::math;
enum class JointType { Prismatic, Revolute, FloatingBase, Nothing };

/*!
 * 计算从A到B的空间坐标变换，B旋转 theta 角度关于axis轴。
 */
template <typename T>
SXform<T> spatialRotation(CoordinateAxis axis, T theta) {
  static_assert(std::is_floating_point<T>::value, "must use floating point value");
  RotMat<T> R = coordinateRotation(axis, theta);
  SXform<T> X = SXform<T>::Zero();
  X.template topLeftCorner<3, 3>() = R;
  X.template bottomRightCorner<3, 3>() = R;
  return X;
}

/*!
 * 计算空间运动叉乘矩阵。
 * 如果可能，优先使用motionCrossProduct。
 */
template <typename T>
Mat6<typename T::Scalar> motionCrossMatrix(const Eigen::MatrixBase<T>& v) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector");
  Mat6<typename T::Scalar> m;
  m << 0, -v(2), v(1), 0, 0, 0, v(2), 0, -v(0), 0, 0, 0, -v(1), v(0), 0, 0, 0, 0,

      0, -v(5), v(4), 0, -v(2), v(1), v(5), 0, -v(3), v(2), 0, -v(0), -v(4), v(3), 0, -v(1), v(0), 0;
  return m;
}

/*!
 * 计算空间力叉乘矩阵。
 * 如果可能，优先使用forceCrossProduct。
 */
template <typename T>
Mat6<typename T::Scalar> forceCrossMatrix(const Eigen::MatrixBase<T>& v) {
  Mat6<typename T::Scalar> f;
  f << 0, -v(2), v(1), 0, -v(5), v(4), v(2), 0, -v(0), v(5), 0, -v(3), -v(1), v(0), 0, -v(4), v(3), 0, 0, 0, 0, 0,
      -v(2), v(1), 0, 0, 0, v(2), 0, -v(0), 0, 0, 0, -v(1), v(0), 0;
  return f;
}

/*!
 * 计算空间运动叉乘。比矩阵乘法版本更快
 */
template <typename T>
SVec<typename T::Scalar> motionCrossProduct(const Eigen::MatrixBase<T>& a, const Eigen::MatrixBase<T>& b) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector");
  SVec<typename T::Scalar> mv;
  mv << a(1) * b(2) - a(2) * b(1), a(2) * b(0) - a(0) * b(2), a(0) * b(1) - a(1) * b(0),
      a(1) * b(5) - a(2) * b(4) + a(4) * b(2) - a(5) * b(1), a(2) * b(3) - a(0) * b(5) - a(3) * b(2) + a(5) * b(0),
      a(0) * b(4) - a(1) * b(3) + a(3) * b(1) - a(4) * b(0);
  return mv;
}

/*!
 * 计算空间力叉乘。比矩阵乘法版本更快
 */
template <typename T>
SVec<typename T::Scalar> forceCrossProduct(const Eigen::MatrixBase<T>& a, const Eigen::MatrixBase<T>& b) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector");
  SVec<typename T::Scalar> mv;
  mv << b(2) * a(1) - b(1) * a(2) - b(4) * a(5) + b(5) * a(4), b(0) * a(2) - b(2) * a(0) + b(3) * a(5) - b(5) * a(3),
      b(1) * a(0) - b(0) * a(1) - b(3) * a(4) + b(4) * a(3), b(5) * a(1) - b(4) * a(2), b(3) * a(2) - b(5) * a(0),
      b(4) * a(0) - b(3) * a(1);
  return mv;
}

/*!
 * 将空间变换转换为齐次坐标变换
 */
template <typename T>
Mat4<typename T::Scalar> sxformToHomogeneous(const Eigen::MatrixBase<T>& X) {
  static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6, "Must have 6x6 matrix");
  Mat4<typename T::Scalar> H = Mat4<typename T::Scalar>::Zero();
  RotMat<typename T::Scalar> R = X.template topLeftCorner<3, 3>();
  Mat3<typename T::Scalar> skewR = X.template bottomLeftCorner<3, 3>();
  H.template topLeftCorner<3, 3>() = R;
  H.template topRightCorner<3, 1>() = matToSkewVec(skewR * R.transpose());
  H(3, 3) = 1;
  return H;
}

/*!
 * 将齐次坐标变换转换为空间变换
 */
template <typename T>
Mat6<typename T::Scalar> homogeneousToSXform(const Eigen::MatrixBase<T>& H) {
  static_assert(T::ColsAtCompileTime == 4 && T::RowsAtCompileTime == 4, "Must have 4x4 matrix");
  Mat3<typename T::Scalar> R = H.template topLeftCorner<3, 3>();
  Vec3<typename T::Scalar> translate = H.template topRightCorner<3, 1>();
  Mat6<typename T::Scalar> X = Mat6<typename T::Scalar>::Zero();
  X.template topLeftCorner<3, 3>() = R;
  X.template bottomLeftCorner<3, 3>() = vectorToSkewMat(translate) * R;
  X.template bottomRightCorner<3, 3>() = R;
  return X;
}

/*!
 * 从旋转矩阵和平移向量创建空间坐标变换
 */
template <typename T, typename T2>
Mat6<typename T::Scalar> createSXform(const Eigen::MatrixBase<T>& R, const Eigen::MatrixBase<T2>& r) {
  static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3, "Must have 3x3 matrix");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 matrix");
  Mat6<typename T::Scalar> X = Mat6<typename T::Scalar>::Zero();
  X.template topLeftCorner<3, 3>() = R;
  X.template bottomRightCorner<3, 3>() = R;
  X.template bottomLeftCorner<3, 3>() = -R * vectorToSkewMat(r);
  return X;
}

/*!
 * 从空间变换中获取旋转矩阵
 */
template <typename T>
RotMat<typename T::Scalar> rotationFromSXform(const Eigen::MatrixBase<T>& X) {
  static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6, "Must have 6x6 matrix");
  RotMat<typename T::Scalar> R = X.template topLeftCorner<3, 3>();
  return R;
}

/*!
 * 从空间变换中获取平移向量
 */
template <typename T>
Vec3<typename T::Scalar> translationFromSXform(const Eigen::MatrixBase<T>& X) {
  static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6, "Must have 6x6 matrix");
  RotMat<typename T::Scalar> R = rotationFromSXform(X);
  Vec3<typename T::Scalar> r = -matToSkewVec(R.transpose() * X.template bottomLeftCorner<3, 3>());
  return r;
}

/*!
 * 反转空间变换（比矩阵逆运算快得多）
 */
template <typename T>
SXform<typename T::Scalar> invertSXform(const Eigen::MatrixBase<T>& X) {
  static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6, "Must have 6x6 matrix");
  RotMat<typename T::Scalar> R = rotationFromSXform(X);
  Vec3<typename T::Scalar> r = -matToSkewVec(R.transpose() * X.template bottomLeftCorner<3, 3>());
  SXform<typename T::Scalar> Xinv = createSXform(R.transpose(), -R * r);
  return Xinv;
}

/*!
 * 计算关节运动子空间向量
 */
template <typename T>
SVec<T> jointMotionSubspace(JointType joint, CoordinateAxis axis) {
  Vec3<T> v(0, 0, 0);
  SVec<T> phi = SVec<T>::Zero();
  if (axis == CoordinateAxis::X) {
    v(0) = 1;
  } else if (axis == CoordinateAxis::Y) {
    v(1) = 1;
  } else {
    v(2) = 1;
  }

  if (joint == JointType::Prismatic) {
    phi.template bottomLeftCorner<3, 1>() = v;
  } else if (joint == JointType::Revolute) {
    phi.template topLeftCorner<3, 1>() = v;
  } else {
    throw std::runtime_error("Unknown motion subspace");
  }

  return phi;
}

/*!
 * 计算关节变换
 */
template <typename T>
Mat6<T> jointXform(JointType joint, CoordinateAxis axis, T q) {
  Mat6<T> X = Mat6<T>::Zero();
  if (joint == JointType::Revolute) {
    X = spatialRotation(axis, q);
  } else if (joint == JointType::Prismatic) {
    Vec3<T> v(0, 0, 0);
    if (axis == CoordinateAxis::X) {
      v(0) = q;
    } else if (axis == CoordinateAxis::Y) {
      v(1) = q;
    } else if (axis == CoordinateAxis::Z) {
      v(2) = q;
    }

    X = createSXform(RotMat<T>::Identity(), v);
  } else {
    throw std::runtime_error("Unknown joint xform\n");
  }
  return X;
}

/*!
 * 构建均匀密度盒子的旋转惯量矩阵
 * @param mass 盒子的质量
 * @param dims 盒子的尺寸
 */
template <typename T>
Mat3<typename T::Scalar> rotInertiaOfBox(typename T::Scalar mass, const Eigen::MatrixBase<T>& dims) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3, "Must have 3x1 vector");
  Mat3<typename T::Scalar> I = Mat3<typename T::Scalar>::Identity() * dims.norm() * dims.norm();
  for (int i = 0; i < 3; i++) {
    I(i, i) -= dims(i) * dims(i);
  }
  I = I * mass / 12;
  return I;
}

/*!
 * 从空间速度转换到线速度
 * 使用给定点的空间速度
 */
template <typename T, typename T2>
Vec3<typename T::Scalar> spatialToLinearVelocity(const Eigen::MatrixBase<T>& v, const Eigen::MatrixBase<T2>& x) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 vector");
  Vec3<typename T::Scalar> vsAng = v.template topLeftCorner<3, 1>();
  Vec3<typename T::Scalar> vsLin = v.template bottomLeftCorner<3, 1>();
  Vec3<typename T::Scalar> vLinear = vsLin + vsAng.cross(x);
  return vLinear;
}

/*!
 * 从空间速度转换到角速度
 */
template <typename T>
Vec3<typename T::Scalar> spatialToAngularVelocity(const Eigen::MatrixBase<T>& v) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector");
  Vec3<typename T::Scalar> vsAng = v.template topLeftCorner<3, 1>();
  return vsAng;
}

/*!
 * 计算给定帧的经典加速度
 * 给定空间加速度和速度
 */
template <typename T, typename T2>
Vec3<typename T::Scalar> spatialToLinearAcceleration(const Eigen::MatrixBase<T>& a, const Eigen::MatrixBase<T2>& v) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 6, "Must have 6x1 vector");

  Vec3<typename T::Scalar> acc;
  // classical accleration = spatial linear acc + omega x v
  acc = a.template tail<3>() + v.template head<3>().cross(v.template tail<3>());
  return acc;
}

/*!
 * 计算帧的经典线加速度
 * 给定空间加速度和速度
 */
template <typename T, typename T2, typename T3>
Vec3<typename T::Scalar> spatialToLinearAcceleration(
    const Eigen::MatrixBase<T>& a, const Eigen::MatrixBase<T2>& v, const Eigen::MatrixBase<T3>& x) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6, "Must have 6x1 vector");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 6, "Must have 6x1 vector");
  static_assert(T3::ColsAtCompileTime == 1 && T3::RowsAtCompileTime == 3, "Must have 3x1 vector");

  Vec3<typename T::Scalar> alin_x = spatialToLinearVelocity(a, x);
  Vec3<typename T::Scalar> vlin_x = spatialToLinearVelocity(v, x);

  // 经典加速度 = 空间线加速度 + omega x v
  Vec3<typename T::Scalar> acc = alin_x + v.template head<3>().cross(vlin_x);
  return acc;
}

/*!
 * 应用空间变换到一个点
 */
template <typename T, typename T2>
Vec3<typename T::Scalar> sXFormPoint(const Eigen::MatrixBase<T>& X, const Eigen::MatrixBase<T2>& p) {
  static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6, "Must have 6x6 vector");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 vector");

  Mat3<typename T::Scalar> R = rotationFromSXform(X);
  Vec3<typename T::Scalar> r = translationFromSXform(X);
  Vec3<typename T::Scalar> Xp = R * (p - r);
  return Xp;
}

/*!
 * 将力从一个点转换到空间力
 * @param f : 力
 * @param p : 点
 */
template <typename T, typename T2>
SVec<typename T::Scalar> forceToSpatialForce(const Eigen::MatrixBase<T>& f, const Eigen::MatrixBase<T2>& p) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3, "Must have 3x1 vector");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3, "Must have 3x1 vector");
  SVec<typename T::Scalar> fs;
  fs.template topLeftCorner<3, 1>() = p.cross(f);
  fs.template bottomLeftCorner<3, 1>() = f;
  return fs;
}

/*!
 *刚体惯性张量的6x6空间惯性张量表示
 */
template <typename T>
class SpatialInertia {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /*!
   *从质量、质心和3x3旋转惯性矩阵构造空间惯性
   */
  SpatialInertia(T mass, const Vec3<T>& com, const Mat3<T>& inertia) {
    Mat3<T> cSkew = vectorToSkewMat(com);
    _inertia.template topLeftCorner<3, 3>() = inertia + mass * cSkew * cSkew.transpose();
    _inertia.template topRightCorner<3, 3>() = mass * cSkew;
    _inertia.template bottomLeftCorner<3, 3>() = mass * cSkew.transpose();
    _inertia.template bottomRightCorner<3, 3>() = mass * Mat3<T>::Identity();
  }

  /*!
   *从6x6矩阵构造空间惯性
   */
  explicit SpatialInertia(const Mat6<T>& inertia) { _inertia = inertia; }

  /*!
   *如果没有参数，则为零
   */
  SpatialInertia() { _inertia = Mat6<T>::Zero(); }

  /*!
   *从质量属性向量构造空间惯性
   */
  explicit SpatialInertia(const MassProperties<T>& a) {
    _inertia(0, 0) = a(4);
    _inertia(0, 1) = a(9);
    _inertia(0, 2) = a(8);
    _inertia(1, 0) = a(9);
    _inertia(1, 1) = a(5);
    _inertia(1, 2) = a(7);
    _inertia(2, 0) = a(8);
    _inertia(2, 1) = a(7);
    _inertia(2, 2) = a(6);
    Mat3<T> cSkew = vectorToSkewMat(Vec3<T>(a(1), a(2), a(3)));
    _inertia.template topRightCorner<3, 3>() = cSkew;
    _inertia.template bottomLeftCorner<3, 3>() = cSkew.transpose();
    _inertia.template bottomRightCorner<3, 3>() = a(0) * Mat3<T>::Identity();
  }

  /*!
   *从伪惯性构造空间惯性。这在Linear Matrix Inequalities for Physically Consistent Inertial Parameter
   *   Identification: A Statistical Perspective on the Mass Distribution中被描述了，作者是Wensing, Kim, Slotine
   *@param P
   */
  explicit SpatialInertia(const Mat4<T>& P) {
    Mat6<T> I;
    T m = P(3, 3);
    Vec3<T> h = P.template topRightCorner<3, 1>();
    Mat3<T> E = P.template topLeftCorner<3, 3>();
    Mat3<T> Ibar = E.trace() * Mat3<T>::Identity() - E;
    I.template topLeftCorner<3, 3>() = Ibar;
    I.template topRightCorner<3, 3>() = vectorToSkewMat(h);
    I.template bottomLeftCorner<3, 3>() = vectorToSkewMat(h).transpose();
    I.template bottomRightCorner<3, 3>() = m * Mat3<T>::Identity();
    _inertia = I;
  }

  /*!
   *将空间惯性转换为质量属性向量
   */
  MassProperties<T> asMassPropertyVector() {
    MassProperties<T> a;
    Vec3<T> h = matToSkewVec(_inertia.template topRightCorner<3, 3>());
    a << _inertia(5, 5), h(0), h(1), h(2), _inertia(0, 0), _inertia(1, 1), _inertia(2, 2), _inertia(2, 1),
        _inertia(2, 0), _inertia(1, 0);
    return a;
  }

  /*!
   * 获取6x6空间惯性矩阵
   */
  const Mat6<T>& getMatrix() const { return _inertia; }

  void setMatrix(const Mat6<T>& mat) { _inertia = mat; }

  void addMatrix(const Mat6<T>& mat) { _inertia += mat; }

  /*!
   * 获取质量
   */
  T getMass() { return _inertia(5, 5); }

  /*!
   * 获取质心位置
   */
  Vec3<T> getCOM() {
    T m = getMass();
    Mat3<T> mcSkew = _inertia.template topRightCorner<3, 3>();
    Vec3<T> com = matToSkewVec(mcSkew) / m;
    return com;
  }

  /*!
   * 获取3x3 旋转惯性矩阵
   */
  Mat3<T> getInertiaTensor() {
    T m = getMass();
    Mat3<T> mcSkew = _inertia.template topRightCorner<3, 3>();
    Mat3<T> I_rot = _inertia.template topLeftCorner<3, 3>() - mcSkew * mcSkew.transpose() / m;
    return I_rot;
  }

  /*!
   * 将其转换为4x4伪惯性矩阵。 这在
   * 线性矩阵不等式中用于物理一致惯性参数
   *   identification：一种统计视角下的质量分布， by
   *   Wensing, Kim, Slotine
   */
  Mat4<T> getPseudoInertia() {
    Vec3<T> h = matToSkewVec(_inertia.template topRightCorner<3, 3>());
    Mat3<T> Ibar = _inertia.template topLeftCorner<3, 3>();
    T m = _inertia(5, 5);
    Mat4<T> P;
    P.template topLeftCorner<3, 3>() = 0.5 * Ibar.trace() * Mat3<T>::Identity() - Ibar;
    P.template topRightCorner<3, 1>() = h;
    P.template bottomLeftCorner<1, 3>() = h.transpose();
    P(3, 3) = m;
    return P;
  }

  /*!
   * 沿着轴翻转惯性矩阵。 这不是很高效，但它工作！
   */
  SpatialInertia flipAlongAxis(CoordinateAxis axis) {
    Mat4<T> P = getPseudoInertia();
    Mat4<T> X = Mat4<T>::Identity();
    if (axis == CoordinateAxis::X) {
      X(0, 0) = -1;
    } else if (axis == CoordinateAxis::Y) {
      X(1, 1) = -1;
    } else if (axis == CoordinateAxis::Z) {
      X(2, 2) = -1;
    }
    P = X * P * X;
    return SpatialInertia(P);
  }

private:
  Mat6<T> _inertia;
};

}  // namespace spatial

#endif  // LIBBIOMIMETICS_SPATIAL_H
