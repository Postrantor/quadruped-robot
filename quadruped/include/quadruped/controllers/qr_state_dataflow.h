/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_STATE_DATAFLOW_H
#define QR_STATE_DATAFLOW_H

#include <yaml-cpp/yaml.h>

#include "config/qr_config.h"
#include "utils/qr_geometry.h"
#include "utils/qr_se3.h"
#include "utils/qr_visualization.h"

struct qrUserParameters {
  /**
   * @brief Constructor of class qrUserParameters.
   * @param file_path: the file path to user parameters.
   */
  qrUserParameters(std::string file_path);

  /**
   * @brief Used to go onto stairs.
   * Will be removed later.
   */
  float stairsTime;

  /**
   * @brief Used to go onto stairs.
   * Will be removed later.
   */
  float stairsVel;

  /**
   * @brief Control frequency, generally 500Hz or 1000Hz.
   */
  unsigned int controlFrequency = 500;

  /**
   * @brief The filter window size used in ground estimator.
   */
  unsigned int filterWindowSize = 50;

  /**
   * @brief Accelerometer variance used in velocity estimator. Used in Kalman filter.
   */
  float accelerometerVariance = 0.01f;

  /**
   * @brief Sensor variance used in velocity estimator. Used in Kalman filter.
   */
  float sensorVariance = 0.01f;

  /**
   * @brief Initial variance used in velocity estimator. Used in Kalman filter.
   */
  float initialVariance = 0.01f;

  /**
   * @brief The moving window filter size used in velocity estimator.
   */
  int movingWindowFilterSize = 50;

  /**
   * @brief Desired body height used for swing leg controller.
   */
  float desiredHeight = A1_BODY_HIGHT;

  /**
   * @brief Desired linear velocity of the quadruped.
   */
  Vec3<float> desiredSpeed = {0.f, 0.f, 0.f};

  /**
   * @brief Desired angular velocity at Z Axis.
   */
  float desiredTwistingSpeed = 0.f;

  /**
   * @brief Maximum distance between foot and ground.
   */
  float footClearance = 0.01f;

  /**
   * @brief Friction coefficient. Currently they are constants.
   * Used in stance leg controllers.
   */
  Vec4<float> frictionCoeffs = {0.45f, 0.45f, 0.45f, 0.45f};

  /**
   * @brief Swing Kp for Raibert Heuristic.
   */
  std::map<std::string, std::vector<float>> swingKp;

  /**
   * @brief Whether to compute force in world frame.
   */
  bool computeForceInWorldFrame = true;

  /**
   * @brief Whether to use whole body control after MPC calculation.
   */
  bool useWBC = true;
};

/**
 * @brief The WbcCtrlData class.
 * This class stores desired states.
 */
class qrWbcCtrlData {
public:
  /**
   * @brief Desired body position in world frame.
   */
  Vec3<float> pBody_des;

  /**
   * @brief Desired body velocity in world frame.
   */
  Vec3<float> vBody_des;

  /**
   * @brief Desired body acceleration in world frame.
   */
  Vec3<float> aBody_des;

  /**
   * @brief Desired body roll pitch yaw.
   */
  Vec3<float> pBody_RPY_des;

  /**
   * @brief Desired body angular velocity in world frame.
   */
  Vec3<float> vBody_Ori_des;

  /**
   * @brief Desired foothold position in world frame.
   */
  Vec3<float> pFoot_des[4];

  /**
   * @brief Desired foothold velocity in world frame.
   */
  Vec3<float> vFoot_des[4];

  /**
   * @brief Desired foothold acceleration in world frame.
   */
  Vec3<float> aFoot_des[4];

  /**
   * @brief Desired foothold force in world frame.
   */
  Vec3<float> Fr_des[4];

  /**
   * @brief Current contact state of 4 foothold.
   */
  Vec4<bool> contact_state;

  /**
   * @brief Whether to conduct WBC.
   * If MPC and WBC are conducted in one iteration, this iteration will consume so much time,
   * so if MPC is conducted in this iteration, WBC will be conducted and vice versa.
   */
  bool allowAfterMPC = true;
};

namespace Quadruped {

struct qrStateDataFlow {
  /**
   * @brief Constructor of class qrStateDataFlow.
   */
  qrStateDataFlow();

  /**
   * @brief Foot position in base frame
   */
  Eigen::Matrix<float, 3, 4> footPositionsInBaseFrame;

  /**
   * @brief Foot velocities in base frame
   */
  Eigen::Matrix<float, 3, 4> footVelocitiesInBaseFrame;

  /**
   * @brief Base linear velocity in world frame.
   */
  Vec3<float> baseVInWorldFrame;

  /**
   * @brief Base angular velocity in world frame.
   */
  Vec3<float> baseWInWorldFrame;

  /**
   * @brief Base linear acceleration in base/IMU frame.
   */
  Vec3<float> baseLinearAcceleration;

  /**
   * @brief Foot Jacobian for 4 legs.
   */
  std::vector<Mat3<float>> footJvs;

  /**
   * @brief Estimated foot force in base frame.
   */
  Eigen::Matrix<float, 3, 4> estimatedFootForce;

  /**
   * @brief Estimated moment in base frame.
   */
  Vec3<float> estimatedMoment;

  /**
   * @brief Body height in control frame.
   */
  float heightInControlFrame = 0.27;

  /**
   * @brief Zero Moment Point of the robot.
   */
  Vec3<float> zmp;

  /**
   * @brief Rotation matrix transform vector from base frame to world frame.
   */
  Mat3<float> baseRMat;

  /**
   * @brief Rotation matrix transform vector from world frame to control frame.
   */
  Mat3<float> groundRMat;

  /**
   * @brief Rotation matrix transform vector from base frame to control frame.
   */
  Mat3<float> baseRInControlFrame;

  /**
   * @brief The orientation of control frame.
   */
  Vec4<float> groundOrientation;

  /**
   * @brief Visualization using Matplotlib.
   */
  Visualization2D visualizer;

  /**
   * @brief Saving data for Whole Body Control.
   */
  qrWbcCtrlData wbcData;
};

}  // namespace Quadruped

#endif  // QR_STATE_DATAFLOW_H
