/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_FSM_STATE_PASSIVE_H
#define QR_FSM_STATE_PASSIVE_H

#include "fsm/qr_fsm_state.hpp"

template <typename T>
class qrFSMStatePassive : public qrFSMState<T> {
public:
  /**
   * @brief Constructor of FSM_State_StandUp.
   * @param controlFSMData: pointer to the data this FSM needed.
   */
  qrFSMStatePassive(qrControlFSMData<T> *_controlFSMData);

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
