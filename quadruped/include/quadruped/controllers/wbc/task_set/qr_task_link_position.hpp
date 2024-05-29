/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_TASK_LINK_POSITION_H
#define QR_TASK_LINK_POSITION_H

#include "qr_task.hpp"

template <typename T>
class qrTaskLinkPosition : public qrTask<T> {
public:
  /**
   * @brief Constructor of class qrTaskLinkPosition.
   * @param fbModel: pointer to MIT floating base model.
   * @param link_idx: the link index in robot model.
   * @param virtual_depend: if the link virtual depend.
   */
  qrTaskLinkPosition(const FloatingBaseModel<T> *fb_model, int link_idx, bool virtual_depend = true);

  virtual ~qrTaskLinkPosition() = default;

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
  virtual bool UpdateCommand(const void *des_pos, const DVec<T> &des_vel, const DVec<T> &des_acc);

  /**
   * @see qrTask::UpdateTaskJacobian
   */
  virtual bool UpdateTaskJacobian();

  /**
   * @see qrTask::UpdateTaskJDotQdot
   */
  virtual bool UpdateTaskJDotQdot();

  /**
   * @brief Pointer to MIT floating base model.
   * Used to get Jc and JDotQDot.
   */
  const FloatingBaseModel<T> *fbModel;

  /**
   * @brief The link index of current link position task.
   */
  int linkIndex;

  /**
   * @brief If the link virtual depend. the jacobian will relate to floating base.
   */
  bool virtualDepend;
};

#endif  // QR_TASK_LINK_POSITION_H
