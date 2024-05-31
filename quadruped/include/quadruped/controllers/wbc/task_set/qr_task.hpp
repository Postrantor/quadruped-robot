/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_TASK_H
#define QR_TASK_H

#include "dynamics/floating_base_model.hpp"

#define TK qrTask<T>

template <typename T>
class qrTask {
public:
  /**
   * @brief 任务类的构造函数。
   * @param dim: 任务的维数。
   */
  qrTask(size_t dim) : dimTask(dim), xddotCmd(dim), posErr(dim), desiredVel(dim), desiredAcc(dim) {}

  virtual ~qrTask() = default;

  /**
   * @brief 更新任务约束，包括 Jt、JtDotQdot 和 xddotCmd。
   * @param des_pos: 任务的期望位置。
   * @param des_vel: 任务的期望速度。
   * @param des_acc: 任务的期望加速度。
   * @return 如果更新成功，则返回 true。
   */
  bool UpdateTask(const void *des_pos, const DVec<T> &des_vel, const DVec<T> &des_acc) {
    UpdateTaskJacobian();
    UpdateTaskJDotQdot();
    UpdateCommand(des_pos, des_vel, des_acc);

    return true;
  }

  /**
   * @brief 获取成员 xddotCmd。
   */
  void GetXddotCmd(DVec<T> &xddot_cmd) const { xddot_cmd = this->xddotCmd; }

  /**
   * @brief 获取成员 Jt。
   */
  void GetJt(DMat<T> &Jt) const { Jt = this->Jt; }

  /**
   * @brief 获取成员 JtDotQdot。
   */
  void GetJtDotQdot(DVec<T> &JtDot_Qdot) const { JtDot_Qdot = this->JtDotQdot; }

  /**
   * @brief 获取成员 posErr。
   */
  const DVec<T> &GetPosErr() const { return posErr; }

  /**
   * @brief 获取成员 desiredVel。
   */
  const DVec<T> &GetDesiredVel() const { return desiredVel; }

  /**
   * @brief 获取成员 desiredAcc。
   */
  const DVec<T> &GetDesiredAcc() const { return desiredAcc; }

protected:
  /**
   * @brief 更新期望加速度命令或位置误差，如果需要。
   * @return 如果更新成功，则返回 true。
   */
  virtual bool UpdateCommand(const void *pos_des, const DVec<T> &vel_des, const DVec<T> &acc_des) = 0;

  /**
   * @brief 更新任务雅可比矩阵。
   * @return 如果更新成功，则返回 true。
   */
  virtual bool UpdateTaskJacobian() = 0;

  /**
   * @brief 更新 JtDotQdot。JtDotQdot 通常来自 MIT 浮动基模型。
   * @return 如果更新成功，则返回 true。
   */
  virtual bool UpdateTaskJDotQdot() = 0;

  /**
   * @brief 任务的维数。
   * 如果 dim=3，则机器人需要在所有三个方向上满足命令。
   */
  size_t dimTask;

  /**
   * @brief 优化后的加速度命令。
   * 加速度命令是从期望位置和期望速度的 PD 控制中计算的。
   */
  DVec<T> xddotCmd;

  /**
   * @brief Jt 点乘 q 的导数。
   * 用于零空间投影。
   */
  DVec<T> JtDotQdot;

  /**
   * @brief 任务雅可比矩阵。
   */
  DMat<T> Jt;

  /**
   * @brief 任务的位置误差。
   * 将被用于零空间投影中。
   * 计算自（期望位置/朝向 - 当前位置/朝向）。
   */
  DVec<T> posErr;

  /**
   * @brief 任务的期望速度。
   * 用于 PD 控制中计算加速度命令。
   */
  DVec<T> desiredVel;

  /**
   * @brief 任务的期望加速度。
   * 用于 PD 控制中计算加速度命令。
   */
  DVec<T> desiredAcc;

  /**
   * @brief 配置空间的维数。
   * 包括浮动基的 6 个维数和关节的 12 个维数。
   */
  int dimConfig = 18;
};

#endif  // QR_TASK_H
