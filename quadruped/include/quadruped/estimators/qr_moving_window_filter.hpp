/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_MOVING_WINDOW_FILTER_H
#define QR_MOVING_WINDOW_FILTER_H

#include <cmath>
#include <deque>

#include "Eigen/Dense"
#include "qr_config.h"

#define Nsta 3  // 状态维度
#define Mobs 3  // 观测维度

namespace Quadruped {

/**
 * @brief 对一序列噪音测量数据进行滤波。
 */
template <class T = double, int N = 1>
class qrMovingWindowFilter {
public:
  /**
   * @brief 类 MovingWindowFilter 的构造函数。
   */
  qrMovingWindowFilter();

  /**
   * @brief 类 MovingWindowFilter 的构造函数。
   * @param windowSize: 移动窗口算法的窗口大小。
   */
  qrMovingWindowFilter(unsigned int windowSize);

  /**
   * @brief 重置方法中的某些中间变量，包括 sum、correction、deque。
   */
  void Reset();

  /**
   * @brief 使用 Neumaier 算法更新移动窗口 sum。
   *        详情请参阅：
   *           https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements
   * @param Args:
   *          value: 要添加到窗口的新值。
   */
  void NeumaierSum(const Eigen::Matrix<T, N, 1> &value);

  /**
   * @brief 将新值推送到窗口队列中，
   * 并计算窗口队列中的平均值。
   * @param newValue: 将新值推送到窗口队列中。
   */
  Eigen::Matrix<T, N, 1> CalculateAverage(const Eigen::Matrix<T, N, 1> &newValue);

  /**
   * @brief 获取成员 sum 的 getter 方法。
   */
  Eigen::Matrix<T, N, 1> GetSum() { return sum; };

private:
  /**
   * @brief 移动窗口算法的窗口大小。
   */
  unsigned int moveWindowSize;

  /**
   * @brief 移动窗口队列中的值之和。
   */
  Eigen::Matrix<T, N, 1> sum;  // 移动窗口 sum。

  /**
   * @brief 用于补偿数值精度损失的校正项。
   */
  Eigen::Matrix<T, N, 1> correction;

  /**
   * @brief 存储移动窗口值。
   */
  std::deque<Eigen::Matrix<T, N, 1>> valueDeque;
};

template <class T, int N>
qrMovingWindowFilter<T, N>::qrMovingWindowFilter() {
  moveWindowSize = DEFAULT_WINDOW_SIZE;
  for (int i = 0; i < N; ++i) {
    sum[i] = 0.;
    correction[i] = 0.;
  }
}

template <class T, int N>
qrMovingWindowFilter<T, N>::qrMovingWindowFilter(unsigned int windowSizeIn) {
  moveWindowSize = windowSizeIn;
  for (int i = 0; i < N; ++i) {
    sum[i] = 0.;
    correction[i] = 0.;
  }
}

template <class T, int N>
void qrMovingWindowFilter<T, N>::Reset() {
  for (int i = 0; i < N; ++i) {
    sum[i] = 0.;
    correction[i] = 0.;
  }
  valueDeque.clear();
}

template <class T, int N>
void qrMovingWindowFilter<T, N>::NeumaierSum(const Eigen::Matrix<T, N, 1> &value) {
  Eigen::Matrix<T, N, 1> newSum = sum + value;
  for (int i = 0; i < N; ++i) {
    if (std::abs(sum[i]) >= std::abs(value[i])) {
      // If self._sum is bigger, low-order digits of value are lost.
      correction[i] += (sum[i] - newSum[i]) + value[i];
    } else {
      // low-order digits of sum are lost
      correction[i] += (value[i] - newSum[i]) + sum[i];
    }
  }

  sum = newSum;
}

/**
 * @brief 在 O(1) 时间内计算移动窗口平均值。
 * @param Args:
 *   new_value: 进入移动窗口的新值。
 * @return 返回：
 *   窗口内值的平均值。
 */
template <class T, int N>
Eigen::Matrix<T, N, 1> qrMovingWindowFilter<T, N>::CalculateAverage(const Eigen::Matrix<T, N, 1> &newValue) {
  int dequeLen = valueDeque.size();
  if (dequeLen >= moveWindowSize) {
    // 从移动窗口 sum 中减去左侧最值。
    NeumaierSum(-valueDeque[0]);
    valueDeque.pop_front();
    dequeLen--;
  }

  NeumaierSum(newValue);
  valueDeque.push_back(newValue);
  return (sum + correction) / (dequeLen + 1);
}

}  // Namespace Quadruped

namespace Quadruped {

template <>
class qrMovingWindowFilter<double, 1> {
public:
  /**
   * @brief 类 MovingWindowFilter 的构造函数。
   */
  qrMovingWindowFilter() {
    moveWindowSize = DEFAULT_WINDOW_SIZE;
    sum = 0.;
    correction = 0.;
  };

  /**
   * @brief 类 MovingWindowFilter 的构造函数。
   * @windowSizeIn: 移动窗口算法的窗口大小。
   */
  qrMovingWindowFilter(unsigned int windowSizeIn) {
    moveWindowSize = windowSizeIn;
    sum = 0.;
    correction = 0.;
  };

  /**
   * @brief 重置方法中的某些中间变量，
   * 包括 sum、correction、deque。
   */
  void Reset() {
    sum = 0.;
    correction = 0.;
    valueDeque.clear();
  };

  /**
   * @brief 使用 Neumaier 算法更新移动窗口 sum。
   *        详情请参阅：
   *           https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements
   * @param value: 要添加到窗口的新值。
   */
  void NeumaierSum(const double &value) {
    double newSum = sum + value;
    if (std::abs(sum) >= std::abs(value)) {
      // 如果 sum 比 value 大，则 value 的低位数字丢失。
      correction += (sum - newSum) + value;
    } else {
      // 如果 value 比 sum 大，则 sum 的低位数字丢失。
      correction += (value - newSum) + sum;
    }
    sum = newSum;
  };

  /**
   * @brief 将新值推送到窗口队列中，
   * 并计算窗口队列中的平均值。
   * @param newValue: 将新值推送到窗口队列中。
   */
  double CalculateAverage(const double &newValue) {
    int dequeLen = valueDeque.size();
    if (dequeLen >= moveWindowSize) {
      // 从移动窗口 sum 中减去左侧最值。
      NeumaierSum(-valueDeque[0]);
      valueDeque.pop_front();
      dequeLen--;
    }

    NeumaierSum(newValue);
    valueDeque.push_back(newValue);
    return (sum + correction) / (dequeLen + 1);
  };

  /**
   * @brief 成员 sum 的 getter 方法。
   */
  double GetSum() { return sum; };

private:
  /**
   * @brief 移动窗口算法的窗口大小。
   */
  unsigned int moveWindowSize;

  /**
   * @brief 移动窗口队列中的值之和。
   */
  double sum;

  /**
   * @brief 用于补偿数值精度损失的校正项。
   */
  double correction;

  /**
   * @brief 存储移动窗口值。
   */
  std::deque<double> valueDeque;
};

}  // Namespace Quadruped

#endif  // QR_MOVING_WINDOW_FILTER_H
