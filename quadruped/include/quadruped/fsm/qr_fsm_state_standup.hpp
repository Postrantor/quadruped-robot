/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_FSM_STATE_STANDUP_H
#define QR_FSM_STATE_STANDUP_H

#include "qr_fsm_state.hpp"

template <typename T>
class qrFSMStateStandUp : public qrFSMState<T> {
public:
  /**
   * @brief Constructor of FSM_State_StandUp
   * @param controlFSMData: pointer to the data this FSM needed
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
   * @brief A variable that indicates whether to stand up.
   * If %standup == 1, then the quadruped should stand up.
   * If %standup == 0, then the quadruped should keep current position.
   * If %standup == -1, then the quadruped should sit down.
   */
  int standUp;

  /**
   * @brief Whether the quadruped stands.
   */
  bool isUp;

  /**
   * @brief When quadruped is standing up, it stores the angle after standing up.
   * When quadruped is sitting down, it stores the angle after sitting down.
   */
  Eigen::Matrix<float, 12, 1> motorAngles;
};

#endif  // QR_FSM_STATE_STANDUP_H
