/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include "quadruped/fsm/qr_fsm_state_passive.hpp"

template <typename T>
qrFSMStatePassive<T>::qrFSMStatePassive(qrControlFSMData<T>* _controlFSMData)
    : qrFSMState<T>(_controlFSMData, FSM_StateName::PASSIVE, "PASSIVE") {
  /* Disable the safety checks. Passive state do not need these check. */
  this->checkSafeOrientation = false;
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
void qrFSMStatePassive<T>::OnEnter() {
  this->nextStateName = this->stateName;
  this->transitionData.Zero();
}

template <typename T>
void qrFSMStatePassive<T>::Run() {
  this->_data->legCmd.clear();

  /* Commands should be set to zero in passive state. */
  for (int i = 0; i < NumMotor; ++i) {
    this->_data->legCmd.push_back({0, 0, 0, 0, 0});
  }
}

template <typename T>
FSM_StateName qrFSMStatePassive<T>::CheckTransition() {
  this->nextStateName = this->stateName;

  /* The %fsmMode will be set according to joy RC mode. Check next state by %fsmMode. */
  switch (int(this->_data->quadruped->fsmMode)) {
    case K_PASSIVE:
      break;

    case K_JOINT_PD:
      this->nextStateName = FSM_StateName::JOINT_PD;
      break;

    case K_STAND_UP:
      this->nextStateName = FSM_StateName::STAND_UP;
      break;

    case K_RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << K_PASSIVE << " to "
                << this->_data->quadruped->fsmMode << std::endl;
  }

  return this->nextStateName;
}

template <typename T>
qrTransitionData<T> qrFSMStatePassive<T>::Transition() {
  this->transitionData.done = true;
  return this->transitionData;
}

template <typename T>
void qrFSMStatePassive<T>::OnExit() {
  /* Do noting when exitting passive state. */
}

template class qrFSMStatePassive<float>;
