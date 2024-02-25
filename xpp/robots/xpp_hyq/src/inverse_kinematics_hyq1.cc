/**
 * @brief
 * @date 2024-02-25
 * @copyright Copyright (c) 2017, Alexander W. Winkler. All rights reserved.
 */

#include <xpp_hyq/inverse_kinematics_hyq1.h>

#include <cmath>
#include <iostream>

namespace xpp {

Joints
InverseKinematicsHyq1::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
  Eigen::Vector3d offset_base_to_hip(0.0, 0.0, 0.15);
  Eigen::VectorXd q0 = leg.GetJointAngles(x_B.at(0) + offset_base_to_hip);

  return Joints({q0});
}


} /* namespace xpp */


