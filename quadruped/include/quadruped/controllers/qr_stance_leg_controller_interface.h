/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_STANCE_LEG_CONTROLLER_INTERFACE_H
#define QR_STANCE_LEG_CONTROLLER_INTERFACE_H

#include "controllers/balance_controller/qr_torque_stance_leg_controller.h"
#include "controllers/mpc/qr_mpc_stance_leg_controller.h"

namespace Quadruped {

/**
 * @brief 站立腿控制器接口。
 * 使用 MPC 控制器或力平衡控制器。
 */
class qrStanceLegControllerInterface {
public:
  /**
   * @brief StanceLegControllerInterface 构造函数。
   * @param robot: 机器人指针。
   * @param gaitGenerator: 步态生成器指针。
   * @param stateEstimators: 状态估计容器指针。
   * @param comAdjuster: COM 调整器指针。
   * @param posePlanner: 姿态规划器指针。
   * @param footholdPlanner: 步骤规划器指针。
   * @param userParameters: 用户参数引用。
   * @param configFilepath: 配置文件路径。
   * 用于确定使用哪个站立腿控制器。
   */
  qrStanceLegControllerInterface(
      qrRobot* robot,
      qrGaitGenerator* gaitGenerator,
      qrStateEstimatorContainer* stateEstimators,
      qrComAdjuster* comAdjuster,
      qrPosePlanner* posePlanner,
      qrFootholdPlanner* footholdPlanner,
      qrUserParameters& userParameters,
      std::string configFilepath);

  /**
   * @brief StanceLegControllerInterface 析构函数。
   * 删除力平衡控制器或 MPC 控制器。
   */
  ~qrStanceLegControllerInterface();

  /**
   * @brief 将所需状态命令绑定到将要使用的站立腿控制器。
   * @param DesiredStateCommand 指针
   */
  void BindCommand(qrDesiredStateCommand* desiredStateCommandIn) { c->BindCommand(desiredStateCommandIn); }

  /**
   * @brief 使用当前时间重置站立腿控制器。
   * @param currentTime: 当前时间重置。
   */
  void Reset(float currentTime);

  /**
   * @brief 每个控制循环更新站立腿控制器。
   * @param currentTime: 当前时间更新。
   */
  void Update(float currentTime);

  /**
   * @brief 获取站立腿控制器的电机命令。
   * @return 由 MPC 或力平衡计算的命令和力。
   */
  std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

  /**
   * @brief 具体站立腿控制器指针。
   */
  TorqueStanceLegController* c;

  /**
   * @brief 力平衡控制器指针。
   */
  TorqueStanceLegController* c1;

  /**
   * @brief MPC 控制器指针。
   */
  TorqueStanceLegController* c2;
};

}  // namespace Quadruped

#endif  // QR_STANCE_LEG_CONTROLLER_INTERFACE_H
