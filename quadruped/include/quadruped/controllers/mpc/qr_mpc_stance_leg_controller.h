/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_MPC_STANCE_LEG_CONTROLLER_H
#define QR_MPC_STANCE_LEG_CONTROLLER_H

#include "controllers/balance_controller/qr_torque_stance_leg_controller.h"
#include "controllers/wbc/qr_wbc_locomotion_controller.hpp"
#include "fsm/qr_control_fsm_data.hpp"
#include "qr_mpc_interface.h"

namespace Quadruped {

class MPCStanceLegController : public TorqueStanceLegController {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief 构造函数 MPCStanceLegController。
   * @param robot：机器人指针。
   * @param gaitGenerator：步态生成器指针。
   * @param stateEstimators：状态估算器容器指针。
   * @param comAdjuster：CoM 调整器指针。
   * @param posePlanner：姿态规划器指针。
   * @param footholdPlanner：足端规划器指针。
   * @param userParameters：用户参数引用。
   * @param configFilepath：配置文件路径字符串。
   */
  MPCStanceLegController(
      qrRobot *robot,
      qrGaitGenerator *gaitGenerator,
      qrStateEstimatorContainer *stateEstimators,
      qrComAdjuster *comAdjuster,
      qrPosePlanner *posePlanner,
      qrFootholdPlanner *footholdPlanner,
      qrUserParameters &userParameters,
      std::string configFilepath);

  /**
   * @brief 使用当前时间重置stance腿控制器。
   * @param currentTime：当前时间重置。
   */
  virtual void Reset(float t);

  /**
   * @brief 获取stance腿控制器的电机命令
   * @return 由MPC计算的命令和力。
   */
  virtual std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

  /**
   * @brief 设置MPC问题并解决MPC问题。
   * @param legCommand：MPC问题的输出。
   * 这可能在未来被删除。
   * @param gaitType：当前状态的步态类型。
   * 这可能在未来被删除。
   * @param robotMode：当前机器人模式。
   * 这可能在未来被删除。
   */
  void Run(std::map<int, qrMotorCommand> &legCommand, int gaitType, int robotMode = 0);

private:
  /**
   * @brief 设置MPC问题的期望命令。
   */
  void SetupCommand();

  /**
   * @brief 更新当前迭代的MPC状态。
   * @param robot：机器人指针。
   */
  void UpdateMPC(qrRobot *robot);

  /**
   * @brief 解决MPC问题
   * @param robot：机器人指针。
   */
  void SolveDenseMPC(qrRobot *robot);

  /**
   * @brief 偏航角速度。
   */
  float yawTurnRate = 0.0;

  /**
   * @brief 期望偏航角。
   */
  float yawDesTrue = 0.0;

  /**
   * @brief 期望滚转角。
   */
  float rollDes;

/**
   * @brief 期望俯仰角。
   */
  float pitchDes;

  /**
   * @brief x轴上的期望速度。
   */
  float xVelDes = 0.0;

  /**
   * @brief y轴上的期望速度。
   */
  float yVelDes = 0.0;

  /**
   * @brief 当前身体高度。
   */
  float bodyHeight;

  /**
   * @brief 补偿后的滚转、俯仰和偏航角。
   */
  Vec3<float> rpyComp;

  /**
   * @brief 一次MPC迭代中的迭代次数。
   */
  int iterationsInaMPC;

  /**
   * @brief 一次MPC迭代中的未来步长。
   */
  const int horizonLength;

  /**
   * @brief 一步态周期中的MPC迭代次数。
   */
  int numHorizonL = 1;

  /**
   * @brief 一次MPC中的迭代次数。
   */
  int defaultIterationsInMpc;

  /**
   * @brief 控制器的控制频率。
   */
  float dt;

  /**
   * @brief 一步MPC的时间。
   */
  float dtMPC;

  /**
   * @brief 用于控制MPC更新频率的迭代计数器。
   */
  unsigned long long iterationCounter = 0;

  /**
   * @brief 存储与horizonLength相关的MPC表格。
   */
  Eigen::Matrix<float, Eigen::Dynamic, 4, Eigen::RowMajor> mpcTable;

  /**
   * @brief腿部对地面的力在基础坐标系中。
   */
  Eigen::Matrix<float, 3, 4> f_ff;

  /**
   * @brief 地面对腿部的反作用力在世界坐标系中。
   */
  Eigen::Matrix<float, 3, 4> f;

  /**
   * @brief 世界坐标系中的期望位置。
   */
  Vec3<float> posDesiredinWorld;

  /**
   * @brief 世界坐标系中的期望线速度。
   */
  Vec3<float> vDesWorld;

  /**
   * @brief 世界坐标系中的当前脚部位置。
   */
  Eigen::Matrix<float, 3, 4> pFoot;

  /**
   * @brief MPC轨迹参考。
   */
  float trajAll[20 * 36];

  /**
   * @brief 存储pose和twist权重的Q向量。
   */
  float Q[12];

  /**
   * @brief 一个bool变量，指示MPC是否已经更新。
   */
  bool mpcUpdated = false;

  /**
   * @brief 一个bool变量，指示是否使用 全身控制。
   */
  bool useWBC = false;
};

}  // Namespace Quadruped

#endif  // QR_MPC_STANCE_LEG_CONTROLLER_H
