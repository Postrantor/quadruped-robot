/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_SAFETY_CHECKER_H
#define QR_SAFETY_CHECKER_H

#include <iostream>

#include "fsm/qr_control_fsm_data.hpp"

/**
 * @brief The SafetyChecker handles the checks requested by the ControlFSM.
 */
template <typename T>

class qrSafetyChecker {
public:
  /**
   * @brief constructor of member qrSafetyChecker
   * @param dataIn: data needed for check, usually robot states and hybrid commands.
   */
  qrSafetyChecker(qrControlFSMData<T>* dataIn) : data(dataIn){};

  /**
   * @brief Check robobts' orientation is safe to control.
   * This is a pre-control check.
   * @return Whether it is safe.
   */
  bool CheckSafeOrientation();

  /**
   * @brief Check foot position not to be stepped too far.
   * This is a post-control check.
   * Currently this method is not used. Just return true.
   * @return Whether it is safe.
   */
  bool CheckPDesFoot();

  /**
   * @brief Check force not to be too large.
   * This is a post-control check.
   * @return Whether it is safe.
   */
  bool CheckForceFeedForward();

private:
  /**
   * @brief A pointer to FSM data. The data is a member of FSM.
   */
  qrControlFSMData<T>* data;
};

#endif  // QR_SAFETY_CHECKER_H
