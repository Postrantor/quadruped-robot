/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_BASE_STATE_ESTIMATOR_H
#define QR_BASE_STATE_ESTIMATOR_H

#include "robots/qr_robot.h"

/**
 * @brief All Estimators should inherit from this class.
 * base class of all generic estimator.
 */
class qrBaseStateEstimator {
public:
  /**
   * @brief Update the estimator.
   */
  virtual void Update(float currentTime) = 0;

  /**
   * @brief Reset the estimator.
   */
  virtual void Reset(float currentTime) = 0;

  /**
   * @brief Destructor of the base estimator class.
   */
  virtual ~qrBaseStateEstimator() = default;
};

#endif  // QR_BASE_STATE_ESTIMATOR_H
