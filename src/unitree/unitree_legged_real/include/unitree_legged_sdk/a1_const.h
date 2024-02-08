/**
 * @file a1_const.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef _UNITREE_LEGGED_A1_H_
#define _UNITREE_LEGGED_A1_H_

namespace UNITREE_LEGGED_SDK {

constexpr double a1_Hip_max = 0.802;    // unit:radian ( = 46   degree)
constexpr double a1_Hip_min = -0.802;   // unit:radian ( = -46  degree)
constexpr double a1_Thigh_max = 4.19;   // unit:radian ( = 240  degree)
constexpr double a1_Thigh_min = -1.05;  // unit:radian ( = -60  degree)
constexpr double a1_Calf_max = -0.916;  // unit:radian ( = -52.5  degree)
constexpr double a1_Calf_min = -2.7;    // unit:radian ( = -154.5 degree)

}  // namespace UNITREE_LEGGED_SDK

#endif
