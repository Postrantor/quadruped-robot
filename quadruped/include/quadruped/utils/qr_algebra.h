/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_ALGEBRA_H
#define QR_ALGEBRA_H

#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>

#include "qr_cpptypes.h"

namespace robotics {

namespace math {

template <typename T>
T clip_min(T command, const T& minVal) {
  if (command < minVal) {
    return minVal;
  } else {
    return command;
  }
}

template <typename T>
T clip_max(T command, const T& maxVal) {
  if (command > maxVal) {
    return maxVal;
  } else {
    return command;
  }
}

template <typename T>
T clip(T command, const T& minVal, const T& maxVal) {
  if (command < minVal) {
    return minVal;
  } else if (command > maxVal) {
    return maxVal;
  } else {
    return command;
  }
}

/**
 * @brief 平方一个数字。
 */
template <typename T>
T square(T a) {
  return a * a;
}

/**
 * @brief 两个 Eigen 矩阵几乎相等。
 */
template <typename T, typename T2>
bool almostEqual(const Eigen::MatrixBase<T>& a, const Eigen::MatrixBase<T>& b, T2 tol) {
  long x = T::RowsAtCompileTime;
  long y = T::ColsAtCompileTime;

  if (T::RowsAtCompileTime == Eigen::Dynamic || T::ColsAtCompileTime == Eigen::Dynamic) {
    assert(a.rows() == b.rows());
    assert(a.cols() == b.cols());
    x = a.rows();
    y = a.cols();
  }

  for (long i = 0; i < x; i++) {
    for (long j = 0; j < y; j++) {
      T2 error = std::abs(a(i, j) - b(i, j));
      if (error >= tol) return false;
    }
  }
  return true;
}

/**
 * @brief 两个浮点数几乎相等。
 */
template <typename T>
bool almostEqual(const T& a, const T b, T tol) {
  T error = std::abs(a - b);
  if (error >= tol) {
    return false;
  }
  return true;
}

/**
 * @brief 计算矩阵的伪逆。
 * @param matrix：输入矩阵。
 * @param sigmaThreshold：奇异值的阈值为零。
 * @param invMatrix：输出矩阵。
 */
template <typename T>
void pseudoInverse(const DMat<T>& matrix, double sigmaThreshold, DMat<T>& invMatrix) {
  if (matrix.rows() == 1 && matrix.cols() == 1) {
    invMatrix.resize(1, 1);
    if (matrix.coeff(0, 0) > sigmaThreshold) {
      invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
    } else {
      invMatrix.coeffRef(0, 0) = 0.0;
    }
    return;
  }

  Eigen::JacobiSVD<DMat<T>> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  /* 不确定是否需要 svd.sort()... 可能不需要。 */
  const int nrows(svd.singularValues().rows());
  DMat<T> invS = DMat<T>::Zero(nrows, nrows);
  for (int ii(0); ii < nrows; ++ii) {
    if (svd.singularValues().coeff(ii) > sigmaThreshold) {
      invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
    }
  }
  invMatrix.noalias() = svd.matrixV() * invS * svd.matrixU().transpose();
}

}  // Namespace math

}  // Namespace robotics

template <typename T>
using DenseMatrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

#endif  // QR_ALGEBRA_H
