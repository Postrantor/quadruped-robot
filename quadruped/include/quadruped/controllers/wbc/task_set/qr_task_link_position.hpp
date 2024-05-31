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
   * @brief qrTaskLinkPosition 类的构造函数。
   * @param fb_model: 指向 MIT 浮动基体模型的指针。
   * @param link_idx: 机器人模型中的链接索引。
   * @param virtual_depend: 如果链接 virtaully 依赖。
   */
  qrTaskLinkPosition(const FloatingBaseModel<T> *fb_model, int link_idx, bool virtual_depend = true);

  virtual ~qrTaskLinkPosition() = default;

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
   */
  virtual bool UpdateTaskJDotQdot();

  /**
   * @brief 指向 MIT 浮动基体模型的指针。
   * 用于获取 Jc 和 JDotQDot。
   */
  const FloatingBaseModel<T> *fbModel;

  /**
   * @brief 当前链接位置任务的链接索引。
   */
  int linkIndex;

  /**
   * @brief 如果链接虚拟依赖，雅可比矩阵将与浮动基体相关。
   */
  bool virtualDepend;
};

#endif  // QR_TASK_LINK_POSITION_H
