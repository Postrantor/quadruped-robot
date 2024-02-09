/**
 * @brief
 * @copyright Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
 */

#ifndef _UNITREE_LEGGED_SAFETY_H_
#define _UNITREE_LEGGED_SAFETY_H_

#include "comm.h"
#include "quadruped.h"

namespace UNITREE_LEGGED_SDK {

class Safety {
public:
  Safety(LeggedType type);
  ~Safety();

  // only effect under Low Level control in Position mode
  void PositionLimit(LowCmd&);
  // only effect under Low Level control, input factor:
  // 1~10, means 10%~100% power limit. If you are new, then use 1; if you are familiar, then can try
  // bigger number or even comment this function.
  int PowerProtect(LowCmd&, LowState&, int);
  // default limit is 5 degree
  int PositionProtect(LowCmd&, LowState&, double limit = 0.087);

private:
  // Watt. When limit to 100, you can triger it with 4 hands shaking.
  int WattLimit, Wcount;
  double Hip_max, Hip_min, Thigh_max, Thigh_min, Calf_max, Calf_min;
};

}  // namespace UNITREE_LEGGED_SDK

#endif
