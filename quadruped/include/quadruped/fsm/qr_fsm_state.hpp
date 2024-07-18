/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_FSM_State_H
#define QR_FSM_State_H

#include <stdio.h>

#include "quadruped/action/qr_action.h"
#include "quadruped/fsm/qr_control_fsm_data.hpp"
#include "quadruped/fsm/qr_transition_data.hpp"

/* 对应RC模式的机器人状态。
 * MIT cheetah3 定义的一些状态在我们的项目中没有使用。
 */
#define K_STAND_DOWN -1
#define K_PASSIVE 0
#define K_STAND_UP 1
#define K_BALANCE_STAND 2
#define GAIT_TRANSITION 3  // when K_LOCOMOTION

#define K_LOCOMOTION 4
#define K_LOCOMOTION_TEST 5
#define K_RECOVERY_STAND 6

#define LOCOMOTION_STAND 7  // when K_LOCOMOTION

#define K_VISION 8
#define K_BACKFLIP 9
#define K_FRONTJUMP 11

/* 特定的控制状态。 */
#define K_JOINT_PD 51
#define K_IMPEDANCE_CONTROL 52

#define K_INVALID 100

/**
 * @brief 状态机中的所有FSM状态。
 */
enum class FSM_StateName {
  INVALID,
  PASSIVE,
  JOINT_PD,
  IMPEDANCE_CONTROL,
  STAND_UP,
  STAND_DOWN,
  BALANCE_STAND,
  LOCOMOTION,
  RECOVERY_STAND,
  VISION,
  BACKFLIP,
  FRONTJUMP
};

template <typename T>
class qrFSMState {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief FSM_State的构造函数。
   * @param controlFSMData: 状态所需的数据。通常是一些控制循环中的组件指针。
   * 状态将从这个控制循环中计算命令并传输到这个%controlFSMData中。
   * %controlFSMData是Control_FSM的成员
   * @param stateNameIn: 枚举类型，所构造状态的名称。
   * @param stateStringIn: 字符串类型，状态名称的字符串。
   */
  qrFSMState(qrControlFSMData<T>* controlFSMData, FSM_StateName stateNameIn, std::string stateStringIn);

  /**
   * @brief 进入状态时执行的操作。
   * @attention 纯虚方法，
   */
  virtual void OnEnter() = 0;

  /**
   * @brief 状态的正常行为。
   */
  virtual void Run() = 0;

  /**
   * @brief 检查当前状态的下一个转换。
   * @return 下一个状态的名称
   */
  virtual FSM_StateName CheckTransition() { return FSM_StateName::INVALID; }

  /**
   * @brief 运行转换行为并返回转换完成时的true。
   * @return 是否转换完成
   */
  virtual qrTransitionData<T> Transition() { return transitionData; }

  /**
   * @brief 离开状态时执行的操作。
   */
  virtual void OnExit() = 0;

  /**
   * @brief 打开所有安全检查。
   */
  void TurnOnAllSafetyChecks();

  /**
   * @brief 关闭所有安全检查。
   */
  void TurnOffAllSafetyChecks();

  /**
   * @brief 当前状态的枚举类型。
   */
  FSM_StateName stateName;

  /**
   * @brief 下一个状态的枚举类型。
   */
  FSM_StateName nextStateName;

  /**
   * @brief 当前状态的字符串。
   */
  std::string stateString;

  /**
   * @brief 转换所需的时间。
   */
  T transitionDuration;

  /**
   * @brief 转换开始的时间。
   */
  T tStartTransition;

  /**
   * @brief 检查滚转和俯仰角。
   * 这是一个预控制安全检查。
   */
  bool checkSafeOrientation = false;

  /**
   * @brief 检查脚步距离
   * 这是一个后控制安全检查。
   */
  bool checkPDesFoot = false;

  /**
   * @brief 检查执行的力是否太大。
   * 这是一个后控制安全检查。
   */
  bool checkForceFeedForward = false;

  /**
   * @brief 检查单腿条件
   * 这是一个后控制安全检查。
   */
  bool checkLegSingularity = false;

protected:
  /**
   * @brief 保存所有相关的控制数据。
   */
  qrControlFSMData<T>* _data;

  /**
   * @brief 转换所需的数据。
   */
  qrTransitionData<T> transitionData;

  /**
   * @brief 记录重置状态的时间。
   */
  float resetTime;

  /**
   * @brief 记录自重置状态以来的时间。
   */
  float timeSinceReset;
};

#endif  // QR_FSM_State_H
