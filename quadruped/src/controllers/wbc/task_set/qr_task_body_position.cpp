/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include "controllers/wbc/task_set/qr_task_body_position.hpp"

template <typename T>
qrTaskBodyPosition<T>::qrTaskBodyPosition(const FloatingBaseModel<T>* fb_model) : qrTask<T>(3), fbModel(fb_model) {
  TK::Jt = DMat<T>::Zero(TK::dimTask, this->dimConfig);
  TK::Jt.block(0, 3, 3, 3).setIdentity();
  TK::JtDotQdot = DVec<T>::Zero(TK::dimTask);

  errScale = DVec<T>::Constant(TK::dimTask, 1.);
  Kp = DVec<T>::Constant(TK::dimTask, 30.);
  Kd = DVec<T>::Constant(TK::dimTask, 1.0);
}

template <typename T>
bool qrTaskBodyPosition<T>::UpdateCommand(const void* des_pos, const DVec<T>& des_vel, const DVec<T>& des_acc) {
  Vec3<T>* pos_cmd = (Vec3<T>*)des_pos;
  Vec3<T> link_pos = fbModel->_state.bodyPosition; /* Body position in world frame. */

  Quat<T> quat = fbModel->_state.bodyOrientation;
  Mat3<T> Rot = robotics::math::quaternionToRotationMatrix(quat); /* Rotation matrix from base frame to world frame. */

  SVec<T> curr_vel = fbModel->_state.bodyVelocity;
  curr_vel.tail(3) = Rot.transpose() * curr_vel.tail(3); /* Current velocity in world frame. */

  for (int i = 0; i < 3; ++i) {
    TK::posErr[i] = errScale[i] * ((*pos_cmd)[i] - link_pos[i]);  // kp_kin=1
    TK::desiredVel[i] = des_vel[i];
    TK::desiredAcc[i] = des_acc[i];

    /* PD control to get acceleration command. */
    TK::xddotCmd[i] =
        Kp[i] * ((*pos_cmd)[i] - link_pos[i]) + Kd[i] * (TK::desiredVel[i] - curr_vel[i + 3]) + TK::desiredAcc[i];
    TK::xddotCmd[i] = std::min(std::max(TK::xddotCmd[i], (T)-10), (T)10);
  }
  return true;
}

template <typename T>
bool qrTaskBodyPosition<T>::UpdateTaskJacobian() {
  Quat<T> quat = fbModel->_state.bodyOrientation;
  Mat3<T> Rot = robotics::math::quaternionToRotationMatrix(quat);
  TK::Jt.block(0, 3, 3, 3) = Rot.transpose();
  return true;
}

template <typename T>
bool qrTaskBodyPosition<T>::UpdateTaskJDotQdot() {
  return true;
}

template class qrTaskBodyPosition<float>;
