/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include "fsm/qr_safety_checker.hpp"

template <typename T>
bool qrSafetyChecker<T>::CheckSafeOrientation() {
  if (abs(data->quadruped->GetBaseRollPitchYaw()[0]) >= 0.5 || abs(data->quadruped->GetBaseRollPitchYaw()[1]) >= 0.5) {
    printf("Orientation safety check failed!\n");
    return false;
  } else {
    return true;
  }
}

template <typename T>
bool qrSafetyChecker<T>::CheckPDesFoot() {
  return true;
}

template <typename T>
bool qrSafetyChecker<T>::CheckForceFeedForward() {
  bool safeForceFeedForward = true;

  /* Check commanded torque of every motor.
   * Work as clip function.
   * The value depends on users. */
  for (int leg = 0; leg < 4; leg++) {
    for (int motorId(0); motorId < 3; ++motorId) {
      if (data->legCmd[leg * 3 + motorId].tua > 23) {
        data->legCmd[leg * 3 + motorId].tua = 23;
      } else if (data->legCmd[leg * 3 + motorId].tua < -23) {
        data->legCmd[leg * 3 + motorId].tua = -23;
      }
    }
  }

  return safeForceFeedForward;
}

template class qrSafetyChecker<float>;
