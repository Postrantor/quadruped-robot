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
 * @brief FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */
template <typename T>
class qrFSMStateLocomotion : public qrFSMState<T> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor of qrFSMStateLocomotion
   * @param controlFSMData: pointer to the data this FSM needed
   */
  qrFSMStateLocomotion(qrControlFSMData<T> *control_fsm_data);

  /**
   * @brief Getter method of locomotionController.
   * @return pointer to locomotionController.
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
   * @brief Reset the locomotion controller during locomotion state.
   * @param currentTime: time to reset the locomotion controller
   */
  void Reset(float currentTime) {
    this->resetTime = currentTime;
    this->timeSinceReset = 0;
    locomotionController->Reset();
  }

private:
  /**
   * @brief Switch gait in locomotion state.
   * Usually switch from trot locomotion to walk locomotion.
   * @return success to switch gait
   */
  bool SwitchMode();

  /**
   * @brief Check safty before transition.
   * Currently this function just return true.
   * @return whether locomotion is currently safe
   */
  bool LocomotionSafe();

  /**
   * @brief The quadruped will keep standing for %transitionDuration * 1000 iterations.
   * This method is used for transitioning.
   * @return currently just return true.
   */
  bool StandLoop();

  /**
   * @brief Pointer to LocomotionController.
   */
  Quadruped::qrLocomotionController *locomotionController;

  /**
   * @brief Pointer to Whole Body Controller.
   */
  qrWbcLocomotionController<T> *wbcController;

  /**
   * @brief Pointer to data needed by Whole Body Controller.
   */
  qrWbcCtrlData *wbcData;

  /**
   * @brief Keep tracking of the iterations of the locomotion controller.
   */
  unsigned long iter = 0;
};

#endif  // QR_FSM_STATE_LOCOMOTION_H
