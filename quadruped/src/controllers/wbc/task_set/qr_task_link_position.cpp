/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include "quadruped/controllers/wbc/task_set/qr_task_link_position.hpp"

template <typename T>
qrTaskLinkPosition<T>::qrTaskLinkPosition(const FloatingBaseModel<T> *fb_model, int link_idx, bool virtual_depend)
    : qrTask<T>(3), fbModel(fb_model), linkIndex(link_idx), virtualDepend(virtual_depend) {
  TK::Jt = DMat<T>::Zero(TK::dimTask, this->dimConfig);
  TK::JtDotQdot = DVec<T>::Zero(TK::dimTask);

  errScale = DVec<T>::Constant(TK::dimTask, 1.);
  Kp = DVec<T>::Constant(TK::dimTask, 100.);
  Kd = DVec<T>::Constant(TK::dimTask, 5.);
}

template <typename T>
bool qrTaskLinkPosition<T>::UpdateCommand(const void *des_pos, const DVec<T> &des_vel, const DVec<T> &des_acc) {
  Vec3<T> *pos_cmd = (Vec3<T> *)des_pos; /* des_pos is in world frame. */
  Vec3<T> link_pos;
  link_pos = fbModel->_pGC[linkIndex];
  Vec3<T> v_error;

  for (int i = 0; i < 3; ++i) {
    TK::posErr[i] = errScale[i] * ((*pos_cmd)[i] - link_pos[i]);
    TK::desiredVel[i] = des_vel[i];
    TK::desiredAcc[i] = des_acc[i];
  }

  /* PD control to get acceleration command. */
  for (size_t i(0); i < TK::dimTask; ++i) {
    v_error[i] = TK::desiredVel[i] - fbModel->_vGC[linkIndex][i];
    TK::xddotCmd[i] = Kp[i] * TK::posErr[i] + Kd[i] * v_error[i] + TK::desiredAcc[i];
  }
  return true;
}

template <typename T>
bool qrTaskLinkPosition<T>::UpdateTaskJacobian() {
  /* Get the Jc from the floating base model. */
  TK::Jt = fbModel->_Jc[linkIndex];
  if (!virtualDepend) {
    TK::Jt.block(0, 0, 3, 6) = DMat<T>::Zero(3, 6);
  }
  return true;
}

template <typename T>
bool qrTaskLinkPosition<T>::UpdateTaskJDotQdot() {
  /* Get JtDotQdot from the floating base model. */
  TK::JtDotQdot = fbModel->_Jcdqd[linkIndex];
  return true;
}

template class qrTaskLinkPosition<float>;
