/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_LOCOMOTION_CONTROLLER_H
#define QR_LOCOMOTION_CONTROLLER_H

#include <Eigen/Dense>
#include <iostream>
#include <map>

#include "controllers/qr_stance_leg_controller_interface.h"
#include "controllers/qr_swing_leg_controller.h"
#include "estimators/qr_state_estimator_container.h"
#include "gait/qr_openloop_gait_generator.h"
#include "gait/qr_walk_gait_generator.h"
#include "planner/qr_com_adjuster.h"
#include "planner/qr_pose_planner.h"
#include "robots/qr_robot.h"
#include "utils/qr_cpptypes.h"

namespace Quadruped {

class qrLocomotionController {
public:
  /**
   * @brief 构造函数，初始化 qrLocomotionController 类。
   * @param robot: 机器人指针。
   * @param gaitGenerator: 步态生成器指针，Openloop Gait Generator 或 Walk Gait Generator。
   * @param desiredStateCommand: DesiredStateCommand 指针。
   * @param stateEstimator: 状态估计容器指针。
   * @param comAdjuster: COM 调整器指针。
   * @param posePlanner: 姿态规划器指针。
   * @param swingLegController: swing腿控制器指针。
   * @param stanceLegController: 站立腿控制器接口指针。
   * @param userParameters: 用户参数指针。
   */
  qrLocomotionController(
      qrRobot *robot,
      qrGaitGenerator *gaitGenerator,
      qrDesiredStateCommand *desiredStateCommand,
      qrStateEstimatorContainer *stateEstimator,
      qrComAdjuster *comAdjuster,
      qrPosePlanner *posePlanner,
      qrRaibertSwingLegController *swingLegController,
      qrStanceLegControllerInterface *stanceLegController,
      qrUserParameters *userParameters);

  ~qrLocomotionController() = default;

  /**
   * @brief 重置所有控制器组件。
   */
  void Reset();

  /**
   * @brief 将 desiredStateCommand 绑定到 swingLegController。
   */
  void BindCommand() {
    swingLegController->BindCommand(desiredStateCommand);
    stanceLegController->BindCommand(desiredStateCommand);
  }

  /**
   * @brief 每个控制循环更新控制器组件。
   */
  void Update();

  /**
   * @brief 通过子控制器计算所有电机命令。
   *  @return 控制输出（例如位置/扭矩）所有（12）电机。
   */
  std::tuple<std::vector<qrMotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

  /**
   * @brief 一个函数，同 GetAction()，仅用于 Debug。
   *  @return 控制输出（例如位置/扭矩）所有（12）电机。
   */
  std::tuple<std::vector<qrMotorCommand>, Eigen::Matrix<float, 3, 4>> GetFakeAction();

  /**
   * @brief 获取成员 gaitGenerator。
   */
  inline qrGaitGenerator *GetGaitGenerator() const { return gaitGenerator; }

  /**
   * @brief 获取成员 swingLegController。
   */
  inline qrRaibertSwingLegController *GetSwingLegController() const { return swingLegController; }

  /**
   * @brief 获取成员 stanceLegController。
   */
  inline TorqueStanceLegController *GetStanceLegController() const { return stanceLegController->c; }

  /**
   * @brief 获取成员 posePlanner。
   */
  inline qrPosePlanner *GetPosePlanner() { return posePlanner; }

  /**
   * @brief 获取机器人自重置以来的当前时间
   * @return 机器人自重置以来的当前时间
   */
  double GetTime() { return robot->GetTimeSinceReset(); }

  /**
   * @brief RaibertSwingLegController 指针
   */
  qrRaibertSwingLegController *swingLegController;

  /**
   * @brief StanceLegControllerInterface 指针
   */
  qrStanceLegControllerInterface *stanceLegController;

  /**
   * @brief 表示是否已停止机器人
   */
  bool stop = false;

  int swingSemaphore = 10000000;

  float stopTick = 0;

private:
  /**
   * @brief 指向机器人指针。
   */
  qrRobot *robot;

  /**
   * @brief 指向步态生成器指针。
   */
  qrGaitGenerator *gaitGenerator;

  /**
   * @brief 指向状态估计容器指针。
   */
  qrStateEstimatorContainer *stateEstimator;

  /**
   * @brief 指向 COM 调整器指针。
   */
  qrComAdjuster *comAdjuster;

  /**
   * @brief 指向姿态规划器指针。
   */
  qrPosePlanner *posePlanner;

  /**
   * @brief 指向 DesiredStateCommand 指针。
   */
  qrDesiredStateCommand *desiredStateCommand;

  /**
   * @brief 每个控制循环将发送到 Gazebo/实际四足机器人的命令列表。
   */
  std::vector<qrMotorCommand> action;

  /**
   * @brief 记录四足机器人重置的时间。
   */
  double resetTime;

  /**
   * @brief 记录四足机器人自重置以来的时间。
   */
  double timeSinceReset;
};

}  // Namespace Quadruped

#endif  // QR_LOCOMOTION_CONTROLLER_H
