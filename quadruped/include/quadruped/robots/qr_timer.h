/**
 * @author Robot Motion and Vision Laboratory at East China Normal University, tophill.robotics@gmail.com
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_TIMER_H
#define QR_TIMER_H

#include <assert.h>
#include <ros/ros.h>
#include <stdint.h>
#include <time.h>

class qrTimer {
public:
  /**
   * @brief qrTimer 类的构造函数
   */
  qrTimer() {
    start = clock();
    startTime = (double)start;
  };

  virtual ~qrTimer() = default;

  /**
   * @brief 获取机器人重置以来的时间。
   * @return 重置以来的时间。
   */
  virtual double GetTimeSinceReset() {
    finish = clock();
    double timeSinceReset = (double)(finish - startTime) / CLOCKS_PER_SEC;  // 秒
    return timeSinceReset;
  };

  /**
   * @brief 设置当前时间为开始时间
   */
  virtual double ResetStartTime() {
    start = clock();
    startTime = (double)start;
    return startTime;
  };

private:
  /**
   * @brief 开始时间。
   */
  clock_t start;

  /**
   * @brief 结束时间。
   */
  clock_t finish;

  /**
   * @brief 开始时间
   */
  double startTime;
};

class qrRosTimer : public qrTimer {
public:
  /**
   * @brief qrRosTimer 类的构造函数
   */
  qrRosTimer() { startRos = ros::Time::now().toSec(); };

  virtual ~qrRosTimer() = default;

  /**
   * @see qrTimer::GetTimeSinceReset
   */
  virtual double GetTimeSinceReset() {
    double timeSinceReset = ros::Time::now().toSec() - startRos;
    return timeSinceReset;
  };

  /**
   * @see qrTimer::ResetStartTime
   */
  virtual double ResetStartTime() {
    startRos = ros::Time::now().toSec();
    return startRos;
  };

private:
  /**
   * @brief ROS 中的开始时间。
   */
  double startRos;
};

class qrTimerInterface {
public:
  /**
   * @brief TimerInterface 类的构造函数。
   * @param useRosTimeIn: 是否使用 ROS 时间工具。
   */
  qrTimerInterface(bool useRosTimeIn = false) : useRosTime(useRosTimeIn), timerPtr(nullptr) {
    if (useRosTime) {
      timerPtr = new qrRosTimer();
    } else {
      timerPtr = new qrTimer();
    }

    startTime = 0;
    timeSinceReset = 0;
  };

  /**
   * @brief TimerInterface 的析构函数。
   */
  ~qrTimerInterface() { delete timerPtr; };

  double GetTimeSinceReset() {
    timeSinceReset = timerPtr->GetTimeSinceReset();
    return timeSinceReset;
  };

  void ResetStartTime() { startTime = timerPtr->ResetStartTime(); };

private:
  bool useRosTime;
  qrTimer* timerPtr;
  double startTime;
  double timeSinceReset;
};

#endif  // QR_TIMER_H
