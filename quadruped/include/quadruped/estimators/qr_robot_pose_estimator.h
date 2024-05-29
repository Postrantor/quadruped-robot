/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_ROBOT_POSE_ESTIMATOR_H
#define QR_ROBOT_POSE_ESTIMATOR_H

#include "estimators/qr_ground_surface_estimator.h"
#include "estimators/qr_robot_velocity_estimator.h"
#include "gait/qr_openloop_gait_generator.h"
#include "robots/qr_robot.h"

namespace Quadruped {

/**
 * @brief estimates robot pose, currently the code implemented position in x-axis and y-axis.
 */
class qrRobotPoseEstimator {
public:
  /**
   * @brief Constructor of robot pose estimator.
   * @param robotIn: the robot class for pose estimation.
   * @param gaitGeneratorIn: generate desired gait schedule for locomotion.
   * @param groundEstimatorIn: estimate the 3D plane where the feet contact.
   * @param velocityEstimator: the estimator class for velocity estimation.
   */
  qrRobotPoseEstimator(
      qrRobot *robotIn,
      qrGaitGenerator *gaitGeneratorIn,
      qrGroundSurfaceEstimator *groundEstimatorIn,
      qrRobotVelocityEstimator *velocityEstimator);

  /**
   * @brief Reset estimater
   * @param currentTime: time since the timer started.
   */
  void Reset(float currentTime);

  /**
   * @brief Compute the time intervial for each loop.
   * @param tick: real robot state used for compution time for each loop.
   * @return time intervial for each loop.
   */
  float ComputeDeltaTime(uint32_t tick);

  /**
   * @brief Update the pose of robot.
   * @param currentTime: time since the timer started.
   */
  void Update(float currentTime);

  /**
   * @brief Caculate position of foot toe in control frame, then represent this vector in world frame,
   * is not absolute height of body in world frame.
   * @return estimated robot height.
   */
  float EstimateRobotHeight();

  /**
   * @brief Simply compute the odometry and update the robot pose(x,y,yaw).
   * @param deltaTime:time intervial for each loop.
   */
  void ComputePose(float deltaTime);

  /**
   * @brief Getter method of member estimatedPose.
   */
  const Vec6<float> &GetEstimatedPose() const { return estimatedPose; };

private:
  /**
   * @brief The robot class for pose estimation.
   */
  qrRobot *robot;

  /**
   * @brief Last time stamp when one loop finished.
   */
  uint32_t lastTimestamp;

  /**
   * @brief Esitimaed pose.
   */
  Vec6<float> estimatedPose;

  /**
   * @brief Generate desired gait schedule for locomotion.
   */
  qrGaitGenerator *gaitGenerator;

  /**
   * @brief Estimate the 3D plane where the feet contact.
   */
  qrGroundSurfaceEstimator *groundEstimator;

  /**
   * @brief Velocity estimatior class for pose estimation.
   */
  qrRobotVelocityEstimator *velocityEstimator;
};

}  // Namespace Quadruped

#endif  // QR_ROBOT_POSE_ESTIMATOR_H
