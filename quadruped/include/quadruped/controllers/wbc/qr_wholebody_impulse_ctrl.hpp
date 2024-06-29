/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_WHOLE_BODY_IMPULSE_CTRL_H
#define QR_WHOLE_BODY_IMPULSE_CTRL_H

#include "Array.hh"
#include "QuadProg++.hh"
#include "fsm/qr_control_fsm_data.hpp"
#include "qr_single_contact.hpp"
#include "task_set/qr_task.hpp"

template <typename T>
class qrWBICExtraData {
public:
  qrWBICExtraData() = default;

  ~qrWBICExtraData() = default;

  /**
   * @brief QP 问题的结果。
   */
  DVec<T> optimizedResult;

  /**
   * @brief 优化的反作用力。
   * 来自 MPC 的力加上 QP 问题的力。
   */
  DVec<T> optimalFr;

  /**
   * @brief 浮动基体部分在 QP 问题中的权重。
   */
  DVec<T> weightFb;

  /**
   * @brief 反作用力部分在 QP 问题中的权重。
   */
  DVec<T> weightFr;
};

template <typename T>
class qrWholeBodyImpulseCtrl {
public:
  /**
   * @brief qrWholeBodyImpulseCtrl 类的构造函数。
   * @param dim_qdot: qdot 的维数。
   * @param contact_list: 机器人所有腿部的接触约束。
   * @param task_list: 机器人的任务列表。
   */
  qrWholeBodyImpulseCtrl(
      size_t dim_qdot,
      const std::vector<qrSingleContact<T> *> *contact_list,
      const std::vector<qrTask<T> *> *task_list);

  virtual ~qrWholeBodyImpulseCtrl() = default;

  /**
   * @brief 获取 MIT 浮动基体模型的结果，
   * 尤其是泛化质量矩阵、科里奥利斯力和泛化重力。
   * @param fb_model: MIT 浮动基体模型。
   */
  void GetModelRes(const FloatingBaseModel<T> &fb_model);

  /**
   * @brief 根据任务列表，计算当前机器人配置
   * 并根据机器人动力学公式计算扭矩命令。
   * @param [out] cmd: 输出扭矩命令 для stance leg。
   * @param [in] extraInput: 如果使用额外数据，这个指针指向 WBIC 额外数据结构。
   */
  void MakeTorque(DVec<T> &cmd, void *extraInput = NULL);

private:
  /**
   * @brief 计算动态一致的加权逆矩阵。
   * 只对 full rank fat 矩阵有效。
   * @param [in] J: 输入雅可比矩阵
   * @param [in] Winv: 泛化质量矩阵的逆矩阵。
   * @param [out] Jinv: J 的伪逆矩阵。
   * @param threshold:  threshold for singular values being zero.
   */
  void WeightedInverse(const DMat<T> &J, const DMat<T> &Winv, DMat<T> &Jinv, double threshold = 0.0001);

  /**
   * @brief 设置动态等式约束。
   * @param qddot: qdot 的导数。
   */
  void SetEqualityConstraint(const DVec<T> &qddot);

  /**
   * @brief 设置不等式约束，包括 مخروط和边界约束。
   */
  void SetInequalityConstraint();

  /**
   * @brief 构建与接触相关的矩阵，包括 JC、JCDotQdot 和 UF。
   */
  void ContactBuilding();

  /**
   * @brief 根据泛化坐标的加速度命令，
   * 计算 stance leg 的总扭矩命令通过动力学公式。
   */
  void GetSolution(const DVec<T> &qddot, DVec<T> &cmd);

  /**
   * @brief 设置 QP 问题中目标的权重。
   */
  void SetCost();

  /**
   * @brief 设置 QP 问题中矩阵的大小。
   */
  void SetOptimizationSize();

  /**
   * @brief 浮动基体的维数。通常设置为 6。
   */
  const size_t dimFb;

  /**
   * @brief qdot 的维数。通常设置为 18。
   */
  const size_t dimQdot;

  /**
   * @brief 动作关节的数量。对于四足机器人，这设置为 12。
   */
  const size_t numActJoint;

  /**
   * @brief 浮动基体在动力学方程中的选择矩阵。
   */
  DMat<T> Sf;

  /**
   * @brief 浮动基体模型的泛化质量惯性矩阵。
   */
  DMat<T> A;

  /**
   * @brief 泛化质量惯性矩阵的逆矩阵。
   */
  DMat<T> Ainv;

  /**
   * @brief 浮动基体模型的科里奥利斯和离心力矩阵。
   */
  DVec<T> Coriolis;

  /**
   * @brief 浮动基体模型的泛化重力矩阵。
   */
  DVec<T> Gravity;

  /**
   * @brief 如果 WBIC 获得了 MIT 浮动基体模型的结果，则设置为 true。
   */
  bool settingUpdated;

  /**
   * @brief 存储所有接触约束的列表。
   * 将用于零空间投影。
   */
  const std::vector<qrSingleContact<T> *> *contactList;

  /**
   * @brief 存储所有kinematic 任务的列表，
   * 包括身体方向、身体位置和链路位置。
   */
  const std::vector<qrTask<T> *> *taskLisk;

  /**
   * @brief 最优变量的维数，包括浮动基体和反作用力的维数。
   */
  size_t dimOptimal;

  /**
   * @brief 等式约束的维数。通常设置为 6。
   */
  size_t dimEqConstraint;

  /**
   * @brief 反作用力的维数。
   * 等于接触点数量的 3 倍。
   */
  size_t dimFr;

  /**
   * @brief 不等式约束的维数。
   * 等于接触点数量的 6 倍。
   */
  size_t dimIneqConstraint;

  /**
   * @brief 指向 ExtraData 的指针，存储 QP 问题的权重和结果。
   */
  qrWBICExtraData<T> *extraData;

  /**
   * @brief QP 问题的等式约束矩阵，以 Eigen 形式表示。
   */
  DMat<T> CE;

  /**
   * @brief QP 问题的等式约束线性向量，以 Eigen 形式表示。
   */
  DVec<T> ce0;

  /**
   * @brief QP 问题的不等式约束矩阵，以 Eigen 形式表示。
   */
  DMat<T> CI;

  /**
   * @brief QP 问题的不等式约束线性向量，以 Eigen 形式表示。
   */
  DVec<T> ci0;

  /**
   * @brief 用于初始加速度命令计算的单位矩阵。
   */
  DMat<T> identityMat;

  /**
   * @brief 不等式约束矩阵的力约束段。
   * @see qrSingleContact::Uf
   */
  DMat<T> UF;

  /**
   * @brief 不等式约束矩阵的线性向量。
   * @see qrSingleContact::ineqVec
   */
  DVec<T> ineqVec;

  /**
   * @brief 包括所有单个接触 Jacobian 的栈式接触 Jacobian。
   * @see qrSingleContact::Jc
   */
  DMat<T> JC;

  /**
   * @brief 包括所有单个 JcDotQdot 的栈式 JcDotQdot。
   * @see qrSingleContact::JcDotQdot
   */
  DVec<T> JCDotQdot;

  /**
   * @brief 由 MPC 解算器计算的期望反作用力。
   */
  DVec<T> desiredFr;

  /**
   * @brief QP 问题的结果向量，以 quadprogpp 形式表示。
   */
  quadprogpp::Vector<double> qpz;

  /**
   * @brief QP 问题的 Hessian 矩阵，以 quadprogpp 形式表示。
   */
  quadprogpp::Matrix<double> qpG;

  /**
   * @brief QP 问题的梯度向量，以 quadprogpp 形式表示。
   */
  quadprogpp::Vector<double> qpg0;

  /**
   * @brief QP 问题的等式约束矩阵，以 quadprogpp 形式表示。
   */
  quadprogpp::Matrix<double> qpCE;

  /**
   * @brief QP 问题的等式约束线性向量，以 quadprogpp 形式表示。
   */
  quadprogpp::Vector<double> qpce0;

  /**
   * @brief QP 问题的不等式约束矩阵，以 quadprogpp 形式表示。
   */
  quadprogpp::Matrix<double> qpCI;

  /**
   * @brief QP 问题的不等式约束线性向量，以 quadprogpp 形式表示。
   */
  quadprogpp::Vector<double> qpci0;
};

#endif  // QR_WHOLE_BODY_IMPULSE_CTRL_H
