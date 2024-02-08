/**
 * @file aliengo_const.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef _UNITREE_LEGGED_ALIENGO_H_
#define _UNITREE_LEGGED_ALIENGO_H_

namespace UNITREE_LEGGED_SDK {

constexpr double aliengo_Hip_max = 1.047;     // unit:radian ( = 60   degree)
constexpr double aliengo_Hip_min = -0.873;    // unit:radian ( = -50  degree)
constexpr double aliengo_Thigh_max = 3.927;   // unit:radian ( = 225  degree)
constexpr double aliengo_Thigh_min = -0.524;  // unit:radian ( = -30  degree)
constexpr double aliengo_Calf_max = -0.611;   // unit:radian ( = -35  degree)
constexpr double aliengo_Calf_min = -2.775;   // unit:radian ( = -159 degree)

}  // namespace UNITREE_LEGGED_SDK

#endif