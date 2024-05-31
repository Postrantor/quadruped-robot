/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_SINGLE_CONTACT_H
#define QR_SINGLE_CONTACT_H

#include "dynamics/floating_base_model.hpp"

template <typename T>
class qrSingleContact {
public:
  /**
   * @brief qrSingleContact 类的构造函数。
   * @param robot: 浮动基体模型类，具有旋翼和接触点。无状态概念。
   * @param contact_pt: 接触点的数量。
   */
  qrSingleContact(FloatingBaseModel<T>* robot, int contact_pt);

  virtual ~qrSingleContact() = default;

  /**
   * @brief 更新接触雅可比矩阵和不等价约束向量。
   * @return 如果更新成功则返回 true。
   */
  bool UpdateContactSpec();

  /**
   * @brief 获取成员 Uf 的行数/维度。
   */
  size_t GetDimUf() const { return Uf.rows(); }

  /**
   * @brief 获取成员 dimContact 的 getter 方法。
   */
  size_t GetDimContact() const { return dimContact; }

  /**
   * @brief 获取成员 Jc 的 getter 方法。
   */
  void GetJc(DMat<T>& Jc) const { Jc = this->Jc; }

  /**
   * @brief 获取成员 JcDotQDot 的 getter 方法。
   */
  void GetJcDotQdot(DVec<T>& JcDotQdot) const { JcDotQdot = this->JcDotQdot; }

  /**
   * @brief 获取成员 Uf 的 getter 方法。
   */
  void GetUf(DMat<T>& Uf) const { Uf = this->Uf; }

  /**
   * @brief 获取成员 ineqVec 的 getter 方法。
   */
  void GetIneqVec(DVec<T>& ineqVec) const { ineqVec = this->ineqVec; }

  /**
   * @brief 获取成员 desiredFr 的 getter 方法。
   */
  const DVec<T>& GetDesiredFr() const { return desiredFr; }

  /**
   * @brief 设置成员 desiredFr 的 setter 方法。
   */
  void SetDesiredFr(const DVec<T>& desiredFr) { this->desiredFr = desiredFr; }

protected:
  /**
   * @brief 通过读取 MIT 浮动基体模型来更新 Jc。
   * @return 如果更新成功则返回 true。
   */
  bool UpdateJc();

  /**
   * @brief 通过读取 MIT 浮动基体模型来更新 JcDotQdot。
   * @return 如果更新成功则返回 true。
   */
  bool UpdateJcDotQdot();

  /**
   * @brief 更新 Uf 矩阵。
   * 目前该矩阵是常量，所以该方法只是返回 true。
   * @todo 考虑添加动态地形信息到 UpdateUf() 中。
   * @return 如果更新成功则返回 true。
   */
  bool UpdateUf();

  /**
   * @brief 更新 ineqVec。
   * 目前只限制 Z 轴方向的力不超过四足机器人的重量，否则将开始 bumping。
   * @return 如果更新成功则返回 true。
   */
  bool UpdateIneqVec();

  /**
   * @brief 指向 MIT 浮动基体模型的指针。
   * 用于获取 Jc 和 JDotQDot。
   */
  FloatingBaseModel<T>* fbModel;

  /**
   * @brief 接触点的最大 z 轴方向的力。
   */
  T maxFz;

  /**
   * @brief 当前的接触点索引。
   * 0, 1, 2, 3 分别对应 FR, FL, RR, RL。
   */
  int indexContact;

  /**
   * @brief 约束矩阵的维度。
   * 通常设置为 6，包括 4 个圆锥约束和 2 个边界约束。
   */
  int dimU;

  /**
   * @brief 地形的摩擦系数。
   * @todo 考虑添加动态地形信息到 mu 中。
   */
  T mu;

  /**
   * @brief 力向量中的 z-力索引。
   * 通常设置为 2。
   */
  int indexFz;

  /**
   * @brief 6x3 不等价约束矩阵，包括圆锥和边界约束。
   */
  DMat<T> Uf;

  /**
   * @brief 所需的反应力。
   * 这是从 MPC 解算器中获得的结果。
   */
  DVec<T> desiredFr;

  /**
   * @brief 6x1 不等价向量。
   * 第六个条目设置为 -maxFz，以满足 fz < maxFz。
   */
  DVec<T> ineqVec;

  /**
   * @brief 单个接触点的雅可比矩阵。
   */
  DMat<T> Jc;

  /**
   * @brief Jc 的导数乘以 q 的导数。
   * 用于 null 空间投影。
   */
  DVec<T> JcDotQdot;

  /**
   * @brief 接触点的维度。
   * 通常设置为 3（x, y, z）。
   */
  size_t dimContact;
};

#endif  // QR_SINGLE_CONTACT_H
