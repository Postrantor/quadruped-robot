/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_WBC_LOCOMOTION_CONTROLLER_H
#define QR_WBC_LOCOMOTION_CONTROLLER_H

#include "controllers/qr_desired_state_command.hpp"
#include "qr_multitask_projection.hpp"
#include "qr_wholebody_impulse_ctrl.hpp"
#include "robots/qr_robot.h"
#include "task_set/qr_task_body_orientation.hpp"
#include "task_set/qr_task_body_position.hpp"
#include "task_set/qr_task_link_position.hpp"

template <typename T>
class qrWbcLocomotionController {
public:
  /**
   * @brief Constructor of class qrWbcLocomotionController .
   * @param fb_model: the MIT floating base model.
   * @param control_fsm_data: some information of robot, like gait and estimators.
   */
  qrWbcLocomotionController(FloatingBaseModel<T> &fb_model, qrControlFSMData<T> *control_fsm_data);

  /**
   * @brief Destructor of class qrWbcLocomotionController .
   */
  ~qrWbcLocomotionController();

  /**
   * @brief Compute desired joint position and velocity using null-space projection,
   * then caculate the desired torque by QP formulation and make it into torque conmmands.
   * @param precomputeData: pointer to qrWbcCtrlData @see qrWbcLocomotionCtrl::wbcCtrlData
   */
  void Run(void *precomputeData);

protected:
  /**
   * @brief Update the floating base model dynamics.
   * @param robot: the robot class for update.
   */
  void UpdateModel(Quadruped::qrRobot *robot);

  /**
   * @brief Update the task classes, let the leg joint.
   * statisfied the reaction force if the leg stance.
   * Maybe the controlFSMData need to be deleted.
   * @param ctrlData: pointer to wbcCtrlData, desired state of the robot.
   * @param controlFSMData: pointer to controlFSMData that stores some information of robot like gait and estimators.
   */
  void ContactTaskUpdate(qrWbcCtrlData *ctrl_data, qrControlFSMData<T> *control_fsm_data);

  /**
   * @brief Write the torque into member jointTorqueCmd.
   * @param control_fsm_data: stores desired command of robot
   */
  void UpdateLegCMD(qrControlFSMData<T> *control_fsm_data);

  /**
   * @brief Task that set the body position. Used in multi-task.
   * @see qrTaskBodyOrientation
   */
  qrTask<T> *taskBodyPos;

  /**
   * @brief Task that set the body orientation. Used in multi-task.
   * @see qrTaskBodyOrientation
   */
  qrTask<T> *taskBodyOri;

  /**
   * @brief Task that set foothold position. Used in multi-task.
   * @see qrTaskLinkPosition.
   */
  qrTask<T> *taskFootPos[4];

  /**
   * @brief Foot contact states.
   * @see qrSingleContact.
   */
  qrSingleContact<T> *footContact[4];

  /**
   * @brief Contact constraints. The list includes contact constraints of the current contact footholds.
   */
  std::vector<qrSingleContact<T> *> contactList;

  /**
   * @brief Prioritized tasks including body orientation and position and link positions.
   */
  std::vector<qrTask<T> *> taskList;

  /**
   * @brief Dimension of configuration, 6 floating base and 12 joints.
   */
  const size_t dimConfig;

  /**
   * @brief Stores desired robot state and MPC results.
   */
  qrWbcCtrlData *wbcCtrlData;

  /**
   * @brief Pointer to controlFSMData, which stores some information like gait and estimators.
   */
  qrControlFSMData<T> *controlFSMData;

  /**
   * @brief Compute position, velocity and acceleration commands with null space projection.
   * @see qrMultitaskProjection
   */
  qrMultitaskProjection<T> *multitask;

  /**
   * @brief Whole body impulse controller which is used to calculate a relaxation target.
   */
  qrWholeBodyImpulseCtrl<T> *wbic;

  /**
   * @brief Extra data, including output of QP problem and weight for QP formulation.
   */
  qrWBICExtraData<T> *wbicExtraData;

  /**
   * @brief Pointer to MIT floating base model.
   * Used to get mass matrix, coriolis matrix and other information used in multitask.
   */
  FloatingBaseModel<T> &fbModel;

  /**
   * @brief The state of a floating base model, mainly kinematic information.
   */
  FBModelState<T> modelState;

  /**
   * @brief Currunt joint states with floating base.
   */
  DVec<T> fullConfig;

  /**
   * @brief Output joint torque commands from WBC control.
   */
  DVec<T> jointTorqueCmd;

  /**
   * @brief Desired joint positions computed from multitask.
   */
  DVec<T> desiredJPos;

  /**
   * @brief Desired joint velocity computed from multitask.
   */
  DVec<T> desiredJVel;

  /**
   * @brief Counter for locomotion loop. Also used to adjust the frequency of WBC calculate.
   */
  unsigned long long iteration;

  /**
   * @brief 3 dim zero vector
   */
  Vec3<T> zeroVec3;
};

#endif  // QR_WBC_LOCOMOTION_CONTROLLER_H
