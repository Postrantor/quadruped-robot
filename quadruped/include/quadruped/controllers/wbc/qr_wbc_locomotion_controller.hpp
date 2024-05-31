/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_WBC_LOCOMOTION_CONTROLLER_H
#define QR_WBC_LOCOMOTION_CONTROLLER_H

#include "controllers/qr_desired_state_command.hpp"
#include "qr_multitask_projection.hpp"
#include "qr_wholebody_impulse_ctrl.hpp"
#include "robots/qr_robot.h"
#include "task_set/qr_task_body_orientation.hpp"
#include "task_set/qr_task_body_position.hpp"
#include "task_set/qr_task_link_position.hpp"

template <typename T>
class qrWbcLocomotionController {
public:
  /**
   * @brief qrWbcLocomotionController 类的构造函数。
   * @param fb_model: MIT 浮动基体模型。
   * @param control_fsm_data: 机器人的一些信息，例如步态和估计器。
   */
  qrWbcLocomotionController(FloatingBaseModel<T> &fb_model, qrControlFSMData<T> *control_fsm_data);

  /**
   * @brief qrWbcLocomotionController 类的析构函数。
   */
  ~qrWbcLocomotionController();

  /**
   * @brief 使用 null 空间投影计算所需的关节位置和速度，然后使用 QP 形式计算所需的扭矩并将其转换为扭矩命令。
   * @param precomputeData: 指向 qrWbcCtrlData 的指针 @see qrWbcLocomotionCtrl::wbcCtrlData
   */
  void Run(void *precomputeData);

protected:
  /**
   * @brief 更新浮动基体模型动力学。
   * @param robot: 用于更新的机器人类。
   */
  void UpdateModel(Quadruped::qrRobot *robot);

  /**
   * @brief 更新任务类，让腿部关节满足反应力，如果腿部是站立的。
   * 可能需要删除 controlFSMData。
   * @param ctrlData: 指向 wbcCtrlData 的指针，机器人的所需状态。
   * @param controlFSMData: 指向 controlFSMData 的指针，该指针存储机器人的一些信息，例如步态和估计器。
   */
  void ContactTaskUpdate(qrWbcCtrlData *ctrl_data, qrControlFSMData<T> *control_fsm_data);

  /**
   * @brief 将扭矩写入成员 jointTorqueCmd 中。
   * @param control_fsm_data: 存储机器人的所需命令。
   */
  void UpdateLegCMD(qrControlFSMData<T> *control_fsm_data);

  /**
   * @brief 设置身体位置的任务。用于多任务中。
   * @see qrTaskBodyOrientation
   */
  qrTask<T> *taskBodyPos;

  /**
   * @brief 设置身体方向的任务。用于多任务中。
   * @see qrTaskBodyOrientation
   */
  qrTask<T> *taskBodyOri;

  /**
   * @brief 设置 foothold 位置的任务。用于多任务中。
   * @see qrTaskLinkPosition
   */
  qrTask<T> *taskFootPos[4];

  /**
   * @brief 足部接触状态。
   * @see qrSingleContact。
   */
  qrSingleContact<T> *footContact[4];

  /**
   * @brief 接触约束。列表包括当前接触 foothold 的接触约束。
   */
  std::vector<qrSingleContact<T> *> contactList;

  /**
   * @brief 优先级任务，包括身体方向、位置和链路位置。
   */
  std::vector<qrTask<T> *> taskList;

  /**
   * @brief 配置维数，6个浮动基体和 12 个关节。
   */
  const size_t dimConfig;

  /**
   * @brief 存储机器人所需状态和 MPC 结果。
   */
  qrWbcCtrlData *wbcCtrlData;

  /**
   * @brief 指向 controlFSMData 的指针，该指针存储一些信息，例如步态和估计器。
   */
  qrControlFSMData<T> *controlFSMData;

  /**
   * @brief 使用 null 空间投影计算位置、速度和加速度命令。
   * @see qrMultitaskProjection
   */
  qrMultitaskProjection<T> *multitask;

  /**
   * @brief 整体身体冲量控制器，用于计算relaxation目标。
   */
  qrWholeBodyImpulseCtrl<T> *wbic;

  /**
   * @brief 额外数据，包括 QP 问题的输出和 QP 形式的权重。
   */
  qrWBICExtraData<T> *wbicExtraData;

  /**
   * @brief 指向 MIT 浮动基体模型的指针。
   * 用于获取质量矩阵、科里奥利斯矩阵和 multitask 中使用的其他信息。
   */
  FloatingBaseModel<T> &fbModel;

  /**
   * @brief 浮动基体模型的状态，主要是运动信息。
   */
  FBModelState<T> modelState;

  /**
   * @brief 当前的关节状态，包括浮动基体。
   */
  DVec<T> fullConfig;

  /**
   * @brief WBC 控制输出的关节扭矩命令。
   */
  DVec<T> jointTorqueCmd;

  /**
   * @brief 从 multitask 计算出的所需关节位置。
   */
  DVec<T> desiredJPos;

  /**
   * @brief 从 multitask 计算出的所需关节速度。
   */
  DVec<T> desiredJVel;

  /**
   * @brief 足部步态循环的计数器，也用于调整 WBC 计算的频率。
   */
  unsigned long long iteration;

  /**
   * @brief 3 维零向量
   */
  Vec3<T> zeroVec3;
};

#endif  // QR_WBC_LOCOMOTION_CONTROLLER_H
