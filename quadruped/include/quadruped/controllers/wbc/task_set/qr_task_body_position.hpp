/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_TASK_BODY_POSITION_H
#define QR_TASK_BODY_POSITION_H

#include "qr_task.hpp"

template <typename T>
class qrTaskBodyPosition : public qrTask<T> {
public:
  /**
   * @brief Constructor of class qrTaskBodyPosition .
   * @param fbModel: pointer to MIT floating base model.
   */
  qrTaskBodyPosition(const FloatingBaseModel<T> *fb_model);

  virtual ~qrTaskBodyPosition() = default;

  /**
   * @brief A scale factor that will mutiply posErr.
   */
  DVec<T> errScale;

  /**
   * @brief KP for position gains.
   * Used in PD control to get acceleration command.
   */
  DVec<T> Kp;

  /**
   * @brief KP for velocity gains
   * Used in PD control to get acceleration command.
   */
  DVec<T> Kd;

protected:
  /**
   * @see qrTask::UpdateCommand
   */
  virtual bool UpdateCommand(const void *pos_des, const DVec<T> &vel_des, const DVec<T> &acc_des);

  /**
   * @see qrTask::UpdateTaskJacobian
   */
  virtual bool UpdateTaskJacobian();

  /**
   * @see qrTask::UpdateTaskJDotQdot
   * @attention In bod orientation task, just set JDotQdot to zero matrix.
   */
  virtual bool UpdateTaskJDotQdot();

  /**
   * @brief Pointer to MIT floating base model.
   * Used to get Jc and JDotQDot.
   */
  const FloatingBaseModel<T> *fbModel;
};

#endif  // QR_TASK_BODY_POSITION_H
