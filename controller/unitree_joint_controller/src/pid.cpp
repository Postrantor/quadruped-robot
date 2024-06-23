/**
 * @brief
 * @date 2024-02-15
 * @copyright Copyright (c) 2024
 */

#include "unitree_joint_controller/pid.hpp"

namespace unitree_joint_controller {
PID::PID(size_t velocity_rolling_window_size)
    : timestamp_(0.0),
      x_(0.0),
      y_(0.0),
      heading_(0.0),
      linear_(0.0),
      angular_(0.0),
      wheel_separation_(0.0),
      left_wheel_radius_(0.0),
      right_wheel_radius_(0.0),
      left_wheel_old_pos_(0.0),
      right_wheel_old_pos_(0.0),
      velocity_rolling_window_size_(velocity_rolling_window_size),
      linear_accumulator_(velocity_rolling_window_size),
      angular_accumulator_(velocity_rolling_window_size) {}

void PID::init(const rclcpp::Time& time) {
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

bool PID::update(double left_pos, double right_pos, const rclcpp::Time& time) {
  // We cannot estimate the speed with very small time intervals:
  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < 0.0001) {
    return false;  // Interval too small to integrate with
  }

  // Get current wheel joint positions:
  const double left_wheel_cur_pos = left_pos * left_wheel_radius_;
  const double right_wheel_cur_pos = right_pos * right_wheel_radius_;

  // Estimate velocity of wheels using old and current position:
  const double left_wheel_est_vel = left_wheel_cur_pos - left_wheel_old_pos_;
  const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;

  // Update old position with current:
  left_wheel_old_pos_ = left_wheel_cur_pos;
  right_wheel_old_pos_ = right_wheel_cur_pos;

  updateFromVelocity(left_wheel_est_vel, right_wheel_est_vel, time);

  return true;
}

bool PID::updateFromVelocity(double left_vel, double right_vel, const rclcpp::Time& time) {
  const double dt = time.seconds() - timestamp_.seconds();

  // Compute linear and angular diff:
  const double linear = (left_vel + right_vel) * 0.5;
  // Now there is a bug about scout angular velocity
  const double angular = (right_vel - left_vel) / wheel_separation_;

  // Integrate PID:
  integrateExact(linear, angular);

  timestamp_ = time;

  // Estimate speeds using a rolling mean to filter them out:
  linear_accumulator_.accumulate(linear / dt);
  angular_accumulator_.accumulate(angular / dt);

  linear_ = linear_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void PID::updateOpenLoop(double linear, double angular, const rclcpp::Time& time) {
  /// Save last linear and angular velocity:
  linear_ = linear;
  angular_ = angular;

  /// Integrate PID:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  integrateExact(linear * dt, angular * dt);
}

void PID::resetPID() {
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void PID::setWheelParams(double wheel_separation, double left_wheel_radius, double right_wheel_radius) {
  wheel_separation_ = wheel_separation;
  left_wheel_radius_ = left_wheel_radius;
  right_wheel_radius_ = right_wheel_radius;
}

void PID::setVelocityRollingWindowSize(size_t velocity_rolling_window_size) {
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}

void PID::integrateRungeKutta2(double linear, double angular) {
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear * cos(direction);
  y_ += linear * sin(direction);
  heading_ += angular;
}

void PID::integrateExact(double linear, double angular) {
  if (fabs(angular) < 1e-6) {
    integrateRungeKutta2(linear, angular);
  } else {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular;
    x_ += r * (sin(heading_) - sin(heading_old));
    y_ += -r * (cos(heading_) - cos(heading_old));
  }
}

void PID::resetAccumulators() {
  linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace unitree_joint_controller
