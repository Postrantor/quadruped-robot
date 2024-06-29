/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_TASK_BODY_ORIENTATION_H
#define QR_TASK_BODY_ORIENTATION_H

#include "qr_task.hpp"

template <typename T>
class qrTaskBodyOrientation : public qrTask<T> {
public:
  /**
   * @brief qrTaskBodyOrientation 类的构造函数。
   * @param fb_model: 指向 MIT 浮动基体模型的指针。
   */
  qrTaskBodyOrientation(const FloatingBaseModel<T> *fb_model);

  virtual ~qrTaskBodyOrientation() = default;

  /**
   * @brief 一个比例因子，用于乘以 posErr。
   */
  DVec<T> errScale;

  /**
   * @brief 位置增益的 KP。
   * 在 PD 控制中用于获取加速度命令。
   */
  DVec<T> Kp;

  /**
   * @brief 速度增益的 KP。
   * 在 PD 控制中用于获取加速度命令。
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
   * @attention 在体姿态任务中，只将 JDotQdot 设置为零矩阵。
   */
  virtual bool UpdateTaskJDotQdot();

  /**
   * @brief 指向 MIT 浮动基体模型的指针。
   * 用于获取 Jc 和 JDotQDot。
   */
  const FloatingBaseModel<T> *fbModel;
};

#endif  // QR_TASK_BODY_ORIENTATION_H
