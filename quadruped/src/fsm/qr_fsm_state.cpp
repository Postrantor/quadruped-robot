/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include "fsm/qr_fsm_state.hpp"

template <typename T>
qrFSMState<T>::qrFSMState(qrControlFSMData<T>* _controlFSMData, FSM_StateName stateNameIn, std::string stateStringIn)
    : _data(_controlFSMData), stateName(stateNameIn), stateString(stateStringIn) {
  transitionData.Zero();
  transitionDuration = T(0);
  std::cout << "[FSM_State] Initialized FSM state: " << stateStringIn << std::endl;
}

template <typename T>
void qrFSMState<T>::TurnOnAllSafetyChecks() {
  checkPDesFoot = true;
  checkLegSingularity = true;
  checkSafeOrientation = true;
  checkForceFeedForward = true;
}

template <typename T>
void qrFSMState<T>::TurnOffAllSafetyChecks() {
  checkPDesFoot = false;
  checkLegSingularity = false;
  checkSafeOrientation = false;
  checkForceFeedForward = false;
}

template class qrFSMState<float>;
