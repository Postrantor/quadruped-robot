/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_FSM_STATE_PASSIVE_H
#define QR_FSM_STATE_PASSIVE_H

#include "quadruped/fsm/qr_fsm_state.hpp"

template <typename T>
class qrFSMStatePassive : public qrFSMState<T> {
public:
  /**
   * @brief FSM_State_Passive的构造函数。
   * @param _controlFSMData: 指向该FSM所需数据的指针。
   */
  qrFSMStatePassive(qrControlFSMData<T>* _controlFSMData);

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
};

#endif  // QR_FSM_STATE_PASSIVE_H
