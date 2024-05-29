/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_MULTITASK_PROJECTION_H
#define QR_MULTITASK_PROJECTION_H

#include "fsm/qr_control_fsm_data.hpp"
#include "qr_single_contact.hpp"
#include "task_set/qr_task.hpp"

template <typename T>
class qrMultitaskProjection {
public:
  /**
   * @brief Consutructor of class qrMultitaskProjection.
   * @param dim_qdot: dimension of qdot.
   */
  qrMultitaskProjection(size_t dim_qdot);

  ~qrMultitaskProjection() = default;

  /**
   * @brief Compute the contact jacobian and update the configuration.
   * Find desired joint position and velocity for joint PD controller.
   * @param [in] curr_config: currunt joint states.
   * @param [in] task_list: prioritized tasks for robot.
   * @param [in] contact_list: contact constraint for all legs of robot.
   * @param [out] des_pos_cmd: desired joint position.
   * @param [out] des_vel_cmd: desired joint velocity.
   * @return if the work finished successfully.
   */
  bool FindConfiguration(
      const DVec<T> &curr_config,
      const std::vector<qrTask<T> *> &task_list,
      const std::vector<qrSingleContact<T> *> &contact_list,
      DVec<T> &des_pos_cmd,
      DVec<T> &des_vel_cmd);

private:
  /**
   * @brief Compute pseudo inverse of matrix J.
   * @param [in] J: input matrix.
   * @param [out] Jinv: inverse matrix of J.
   */
  void PseudoInverse(const DMat<T> &J, DMat<T> &Jinv);

  /**
   * @brief Compute the null-space projection matrix of J.
   * @param [in] J: previous jacobian matrix.
   * @param [out] N: null-space projection matrix.
   */
  void BuildProjectionMatrix(const DMat<T> &J, DMat<T> &N);

  /**
   * @brief Threshold for singular values being zero.
   * Used in computing the pseudo inverse matrix.
   */
  double thresholdInv;

  /**
   * @brief Dimension of qdot.
   */
  const size_t dimQDot;

  /**
   * @brief Number of actuable joints.
   */
  const size_t numActJoint;

  /**
   * @brief Identity matrix used in computing null-space projection.
   */
  DMat<T> identityMat;

  /**
   * @brief Null-space projection matrix of contact jacobian matrix.
   */
  DMat<T> Nc;
};

#endif  // QR_MULTITASK_PROJECTION_H
