/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_BASE_STATE_ESTIMATOR_H
#define QR_BASE_STATE_ESTIMATOR_H

#include "robots/qr_robot.h"

/**
 * @brief 所有估算器都应该继承自这个类。
 * 所有泛型估算器的基类。
 */
class qrBaseStateEstimator {
public:
  /**
   * @brief 更新估算器。
   */
  virtual void Update(float currentTime) = 0;

  /**
   * @brief 重置估算器。
   */
  virtual void Reset(float currentTime) = 0;

  /**
   * @brief 基础估算器类的析构函数。
   */
  virtual ~qrBaseStateEstimator() = default;
};

#endif  // QR_BASE_STATE_ESTIMATOR_H
