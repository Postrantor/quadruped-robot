/**
 * @brief
 * @date 2024-02-25
 * @copyright Copyright (c) 2017, Alexander W. Winkler. All rights reserved.
 */

#include <xpp_hyq/inverse_kinematics_hyq2.h>

#include <cmath>
#include <iostream>

#include <xpp_states/endeffector_mappings.h>


namespace xpp {

Joints
InverseKinematicsHyq2::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
  using namespace biped;
  std::vector<Eigen::VectorXd> q_vec;

  // make sure always exactly 2 elements
  auto x_biped_B = x_B.ToImpl();
  x_biped_B.resize(2, x_biped_B.front());

  q_vec.push_back(leg.GetJointAngles(x_biped_B.at(L) + Vector3d(0.0, -0.1, 0.15)));
  q_vec.push_back(leg.GetJointAngles(x_biped_B.at(R) + Vector3d(0.0,  0.1, 0.15)));


  return Joints(q_vec);
}

} /* namespace xpp */


