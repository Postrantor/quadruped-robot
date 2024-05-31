/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_FSM_STATE_LOCOMOTION_H
#define QR_FSM_STATE_LOCOMOTION_H

#include "controllers/qr_locomotion_controller.h"
#include "controllers/wbc/qr_wbc_locomotion_controller.hpp"
#include "qr_control_fsm_data.hpp"
#include "qr_fsm_state.hpp"

/**
 * @brief 机器人行走FSM状态。管理联系逻辑和处理控制器接口的调用。
 * 这个状态应该独立于控制器、步态和期望轨迹。
 */
template <typename T>
class qrFSMStateLocomotion : public qrFSMState<T> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief qrFSMStateLocomotion的构造函数
   * @param control_fsm_data: 指向该FSM所需数据的指针
   */
  qrFSMStateLocomotion(qrControlFSMData<T> *control_fsm_data);

  /**
   * @brief 获取行走控制器的方法。
   * @return 指向行走控制器的指针。
   */
  Quadruped::qrLocomotionController *GetLocomotionController() const { return locomotionController; }

  /**
   * @see qrFSMState::OnEnter
   */
  void OnEnter();

  /**
   * @see qrFSMState::Run
   */
  virtual void Run();

  /**
   * @see qrFSMState::CheckTransition
   */
  FSM_StateName CheckTransition();

  /**
   * @see qrFSMState::Transition
   */
  qrTransitionData<T> Transition();

  /**
   * @see qrFSMState::OnExit
   */
  void OnExit();

  /**
   * @brief 在行走状态下重置行走控制器。
   * @param currentTime: 重置行走控制器的时间
   */
  void Reset(float currentTime) {
    this->resetTime = currentTime;
    this->timeSinceReset = 0;
    locomotionController->Reset();
  }

private:
  /**
   * @brief 在行走状态下切换步态。
   * 通常从快步态切换到步态。
   * @return 是否成功切换步态
   */
  bool SwitchMode();

  /**
   * @brief 在过渡前检查安全。
   * 目前这个函数只是返回true。
   * @return 是否当前行走安全
   */
  bool LocomotionSafe();

  /**
   * @brief 机器人将保持站立%transitionDuration * 1000次迭代。
   * 这个方法用于过渡。
   * @return 目前只是返回true。
   */
  bool StandLoop();

  /**
   * @brief 指向行走控制器的指针。
   */
  Quadruped::qrLocomotionController *locomotionController;

  /**
   * @brief 指向整体控制器的指针。
   */
  qrWbcLocomotionController<T> *wbcController;

  /**
   * @brief 指向整体控制器所需数据的指针。
   */
  qrWbcCtrlData *wbcData;

  /**
   * @brief 跟踪行走控制器的迭代次数。
   */
  unsigned long iter = 0;
};

#endif  // QR_FSM_STATE_LOCOMOTION_H
