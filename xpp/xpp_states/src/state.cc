/**
 * @brief
 * @date 2024-02-25
 * @copyright Copyright (c) 2017, Alexander W. Winkler. All rights reserved.
 */

#include <vector>

#include <xpp_states/state.h>

namespace xpp {

StateLinXd::StateLinXd(int dim) {
  kNumDim = dim;
  p_ = VectorXd::Zero(dim);
  v_ = VectorXd::Zero(dim);
  a_ = VectorXd::Zero(dim);
}

StateLinXd::StateLinXd(const VectorXd &_p, const VectorXd &_v,
                       const VectorXd &_a)
    : StateLinXd(_p.rows()) {
  p_ = _p;
  v_ = _v;
  a_ = _a;
}

StateLinXd::StateLinXd(const VectorXd &_p) : StateLinXd(_p.rows()) { p_ = _p; }

const VectorXd StateLinXd::GetByIndex(MotionDerivative deriv) const {
  switch (deriv) {
  case kPos:
    return p_;
    break;
  case kVel:
    return v_;
    break;
  case kAcc:
    return a_;
    break;
  default:
    throw std::runtime_error(
        "[StateLinXd::GetByIndex] derivative not part of state");
  }
}

VectorXd &StateLinXd::GetByIndex(MotionDerivative deriv) {
  switch (deriv) {
  case kPos:
    return p_;
    break;
  case kVel:
    return v_;
    break;
  case kAcc:
    return a_;
    break;
  default:
    throw std::runtime_error(
        "[StateLinXd::GetByIndex] derivative not part of state");
  }
}

StateLin3d::StateLin3d(const StateLinXd &state_xd) : StateLinXd(3) {
  assert(state_xd.kNumDim == 3);

  p_ = state_xd.p_;
  v_ = state_xd.v_;
  a_ = state_xd.a_;
}

StateLin2d StateLin3d::Get2D() const {
  StateLin2d p2d;
  p2d.p_ = p_.topRows<kDim2d>();
  p2d.v_ = v_.topRows<kDim2d>();
  p2d.a_ = a_.topRows<kDim2d>();
  return p2d;
}

Vector6d State3d::Get6dVel() const {
  Vector6d h_xd;
  h_xd.segment(AX, 3) = ang.w;
  h_xd.segment(LX, 3) = lin.v_;
  return h_xd;
}

Vector6d State3d::Get6dAcc() const {
  Vector6d h_xdd;
  h_xdd.segment(AX, 3) = ang.wd;
  h_xdd.segment(LX, 3) = lin.a_;
  return h_xdd;
}

} // namespace xpp
