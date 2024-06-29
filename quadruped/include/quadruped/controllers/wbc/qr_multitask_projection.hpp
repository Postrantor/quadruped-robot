/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
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
   * @brief qrMultitaskProjection 类的构造函数。
   * @param dim_qdot: qdot 的维度。
   */
  qrMultitaskProjection(size_t dim_qdot);

  ~qrMultitaskProjection() = default;

  /**
   * @brief 计算接触雅可比矩阵并更新配置。
   * 为关节 PD 控制器找到所需的关节位置和速度。
   * @param [in] curr_config: 当前的关节状态。
   * @param [in] task_list: 任务列表。
   * @param [in] contact_list:所有腿的接触约束。
   * @param [out] des_pos_cmd: 所需的关节位置。
   * @param [out] des_vel_cmd: 所需的关节速度。
   * @return 如果工作完成成功则返回 true。
   */
  bool FindConfiguration(
      const DVec<T> &curr_config,
      const std::vector<qrTask<T> *> &task_list,
      const std::vector<qrSingleContact<T> *> &contact_list,
      DVec<T> &des_pos_cmd,
      DVec<T> &des_vel_cmd);

private:
  /**
   * @brief 计算矩阵 J 的伪逆。
   * @param [in] J: 输入矩阵。
   * @param [out] Jinv: J 的逆矩阵。
   */
  void PseudoInverse(const DMat<T> &J, DMat<T> &Jinv);

  /**
   * @brief 计算 J 的零空间投影矩阵。
   * @param [in] J: 之前的雅可比矩阵。
   * @param [out] N: 零空间投影矩阵。
   */
  void BuildProjectionMatrix(const DMat<T> &J, DMat<T> &N);

  /**
   * @brief 伪逆矩阵计算中的阈值。
   * 用于计算伪逆矩阵时判断奇异值是否为零。
   */
  double thresholdInv;

  /**
   * @brief qdot 的维度。
   */
  const size_t dimQDot;

  /**
   * @brief 可驱动关节的数量。
   */
  const size_t numActJoint;

  /**
   * @brief 计算零空间投影矩阵时使用的单位矩阵。
   */
  DMat<T> identityMat;

  /**
   * @brief 接触雅可比矩阵的零空间投影矩阵。
   */
  DMat<T> Nc;
};

#endif  // QR_MULTITASK_PROJECTION_H
