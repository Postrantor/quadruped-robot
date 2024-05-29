/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_TRANSITION_DATA_H
#define QR_TRANSITION_DATA_H

#include "robots/qr_robot.h"

/**
 * @brief Struct of relevant data that can be used during transition to pass data between states.
 */
template <typename T>
struct qrTransitionData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  qrTransitionData() { Zero(); }

  /**
   * @brief Zero out all of the data.
   */
  void Zero() {
    /* Flag to mark when transition is done. */
    done = false;

    /* Timing parameters. */
    t0 = 0.0;         /* time that transition started. */
    tCurrent = 0.0;   /* current time since transition started. */
    tDuration = 0.0;  // overall transition duration

    // Robot state at the beginning of transition
    comState0 = Vec12<T>::Zero();  // center of mass state
    qJoints0 = Vec12<T>::Zero();   // joint positions
    pFoot0 = Mat34<T>::Zero();     // foot positions

    // Current robot state
    comState = Vec12<T>::Zero();  // center of mass state
    qJoints = Vec12<T>::Zero();   // joint positions
    pFoot = Mat34<T>::Zero();     // foot positions

    legCommand.clear();
  }

  // Flag to mark when transition is done
  bool done = false;

  /**
   * @brief Time that transition started.
   */
  T t0;

  /**
   * @brief Current time since transition started.
   */
  T tCurrent;

  /**
   * @brief Overall transition duration.
   */
  T tDuration;

  /**
   * @brief Center of mass state.
   */
  Vec12<T> comState0;

  /**
   * @brief Joint positions.
   */
  Vec12<T> qJoints0;

  /**
   * @brief Foot positions.
   */
  Mat34<T> pFoot0;

  /**
   * @brief Center of mass state
   */
  Vec12<T> comState;

  /**
   * @brief Joint positions.
   */
  Vec12<T> qJoints;

  /**
   * @brief Foot positions.
   */
  Mat34<T> pFoot;

  std::vector<Quadruped::qrMotorCommand> legCommand;
};

template struct qrTransitionData<float>;

#endif  // QR_TRANSITION_DATA_H
