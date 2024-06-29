/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_CONTROL_FSM_H
#define QR_CONTROL_FSM_H

#include <iostream>

#include "qr_control_fsm_data.hpp"
#include "qr_fsm_state_locomotion.hpp"
#include "qr_fsm_state_passive.hpp"
#include "qr_fsm_state_standup.hpp"
#include "qr_safety_checker.hpp"

/**
 * @brief Enumerate all of the operating modes.
 */
enum class FSM_OperatingMode { NORMAL, TRANSITIONING, ESTOP, EDAMP };

template <typename T>
struct qrFSMStatesList {
  qrFSMState<T> *invalid;
  qrFSMStatePassive<T> *passive;
  qrFSMStateStandUp<T> *standUp;
  qrFSMStateLocomotion<T> *locomotion;
};

/**
 * @brief 控制FSM处理来自更高级别的FSM状态。
 */
template <typename T>
class qrControlFSM {
public:
  /**
   * @brief qrControlFSM类的构造函数。
   * @param quadruped: 机器人指针。
   * @param stateEstimator: 状态估算器容器指针。
   * @param gaitScheduler: 步态计划生成器指针。
   * @param desiredStateCommand: 期望状态命令指针。
   * @param userParameters: 用户参数指针。
   */
  qrControlFSM(
      Quadruped::qrRobot *quadruped,
      Quadruped::qrStateEstimatorContainer *stateEstimator,
      Quadruped::qrGaitGenerator *gaitScheduler,
      Quadruped::qrDesiredStateCommand *desiredStateCommand,
      qrUserParameters *userParameters);

  ~qrControlFSM() = default;

  /**
   * @brief 初始化ControlFSM。
   * 机器人将进入站立状态。
   */
  void Initialize();

  /**
   * @brief 重置ControlFSM的时间。
   * @param currentTime: 重置ControlFSM的当前时间。
   */
  void Reset(float currentTime) {
    resetTime = currentTime;
    timeSinceReset = 0;
    statesList.locomotion->Reset(currentTime);
  }

  /**
   * @brief 执行ControlFSM一次。它将在一个状态或过渡中。
   * @param hybridAction: 期望电机命令的返回值。
   */
  void RunFSM(std::vector<Quadruped::qrMotorCommand> &hybridAction);

  /**
   * @brief 在计算命令之前检查机器人状态。
   * @todo 将此函数删除或移到SafetyCheck类中。
   * @return 操作模式
   */
  FSM_OperatingMode SafetyPreCheck();

  /**
   * @brief 在计算期望命令后检查机器人状态。
   * @return 操作模式
   */
  FSM_OperatingMode SafetyPostCheck();

  /**
   * @brief 按照所需状态名称获取下一个FSM状态指针。
   * @param stateName: 所需状态的名称。
   * @return FSM状态列表中的指针
   */
  qrFSMState<T> *GetNextState(FSM_StateName stateName);

  /**
   * @brief 获取行走控制器在行走状态下的指针。
   * @return 指向行走控制器的指针
   */
  Quadruped::qrLocomotionController *GetLocomotionController() const {
    return statesList.locomotion->GetLocomotionController();
  }

  /**
   * @brief 打印当前FSM状态
   * @param opt: 打印状态的选项
   */
  void PrintInfo(int opt);

private:
  /**
   * @brief FSM的操作模式。
   */
  FSM_OperatingMode operatingMode;

  /**
   * @brief FSM所需的数据。
   */
  qrControlFSMData<T> data;

  /**
   * @brief 检查输入和计算命令。
   */
  qrSafetyChecker<T> *safetyChecker;

  /**
   * @brief 在过渡时存储数据。
   */
  qrTransitionData<T> transitionData;

  /**
   * @brief 包含所有FSM状态。
   */
  qrFSMStatesList<T> statesList;

  /**
   * @brief 当前的FSM状态。
   */
  qrFSMState<T> *currentState;

  /**
   * @brief 下一个FSM状态。
   */
  qrFSMState<T> *nextState;

  /**
   * @brief 下一个FSM状态的名称。
   */
  FSM_StateName nextStateName;

  /**
   * @brief 重置FSM的时间。
   */
  float resetTime;

  /**
   * @brief 自重置FSM以来经过的时间。
   */
  float timeSinceReset;

  /**
   * @brief 每%printNUM次迭代打印信息。
   */
  int printNum = 10000;

  /**
   * @brief 当前的正常打印计数。
   */
  int printIter = 0;

  /**
   * @brief FSM的当前迭代次数。
   */
  int iter = 0;
};

#endif  // QR_CONTROL_FSM_H
