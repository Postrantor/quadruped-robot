/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_FSM_STATE_STANDUP_H
#define QR_FSM_STATE_STANDUP_H

#include "quadruped/fsm/qr_fsm_state.hpp"

template <typename T>
class qrFSMStateStandUp : public qrFSMState<T> {
public:
  /**
   * @brief FSM_State_StandUp的构造函数。
   * @param controlFSMData: 指向该FSM所需数据的指针。
   */
  qrFSMStateStandUp(qrControlFSMData<T>* controlFSMData);

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

private:
  /**
   * @brief 是否站立的标志变量。
   * 如果%standup == 1，则四足机器人应该站立。
   * 如果%standup == 0，则四足机器人应该保持当前位置。
   * 如果%standup == -1，则四足机器人应该坐下。
   */
  int standUp;

  /**
   * @brief 四足机器人是否站立的标志。
   */
  bool isUp;

  /**
   * @brief 四足机器人站立或坐下时的电机角度。
   * 当四足机器人站立时，存储站立后的电机角度。
   * 当四足机器人坐下时，存储坐下的电机角度。
   */
  Eigen::Matrix<float, 12, 1> motorAngles;
};

#endif  // QR_FSM_STATE_STANDUP_H
