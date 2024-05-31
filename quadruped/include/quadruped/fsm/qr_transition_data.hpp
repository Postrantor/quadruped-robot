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
 * @brief 在状态转换期间传递数据的相关数据结构。
 */
template <typename T>
struct qrTransitionData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  qrTransitionData() { Zero(); }

  /**
   * @brief 将所有数据清零。
   */
  void Zero() {
    /* 标记转换是否完成的标志。 */
    done = false;

    /* 定时参数。 */
    t0 = 0.0;         /* 转换开始的时间。 */
    tCurrent = 0.0;   /* 自转换开始以来的当前时间。 */
    tDuration = 0.0;  // 总的转换持续时间

    // 转换开始时机器人状态
    comState0 = Vec12<T>::Zero();  // 质心状态
    qJoints0 = Vec12<T>::Zero();   // 关节位置
    pFoot0 = Mat34<T>::Zero();     // 脚部位置

    // 当前机器人状态
    comState = Vec12<T>::Zero();  // 质心状态
    qJoints = Vec12<T>::Zero();   // 关节位置
    pFoot = Mat34<T>::Zero();     // 脚部位置

    legCommand.clear();
  }

  // 标记转换是否完成的标志
  bool done = false;

  /**
   * @brief 转换开始的时间。
   */
  T t0;

  /**
   * @brief 自转换开始以来的当前时间。
   */
  T tCurrent;

  /**
   * @brief 总的转换持续时间。
   */
  T tDuration;

  /**
   * @brief 质心状态。
   */
  Vec12<T> comState0;

  /**
   * @brief 关节位置。
   */
  Vec12<T> qJoints0;

  /**
   * @brief 脚部位置。
   */
  Mat34<T> pFoot0;

  /**
   * @brief 质心状态。
   */
  Vec12<T> comState;

  /**
   * @brief 关节位置。
   */
  Vec12<T> qJoints;

  /**
   * @brief 脚部位置。
   */
  Mat34<T> pFoot;

  std::vector<Quadruped::qrMotorCommand> legCommand;
};

template struct qrTransitionData<float>;

#endif  // QR_TRANSITION_DATA_H
