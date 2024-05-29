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
   * @brief Constructor of class qrTimer
   */
  qrTimer() {
    start = clock();
    startTime = (double)start;
  };

  virtual ~qrTimer() = default;

  /**
   * @brief Get time since robot reset.
   * @return time since reset.
   */
  virtual double GetTimeSinceReset() {
    finish = clock();
    double timeSinceReset = (double)(finish - startTime) / CLOCKS_PER_SEC;  // second(s)
    return timeSinceReset;
  };

  /**
   * @brief Set current time as start time
   */
  virtual double ResetStartTime() {
    start = clock();
    startTime = (double)start;
    return startTime;
  };

private:
  /**
   * @brief Start time.
   */
  clock_t start;

  /**
   * @brief Finish time.
   */
  clock_t finish;

  /**
   * @brief Start time
   */
  double startTime;
};

class qrRosTimer : public qrTimer {
public:
  /**
   * @brief Constructor of class qrRosTimer
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
   * @brief Start time in ROS.
   */
  double startRos;
};

class qrTimerInterface {
public:
  /**
   * @brief Constructor of class TimerInterface.
   * @param useRosTimeIn: whether to use ROS timer tools.
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
   * @brief Destructor of qrTimerInterface.
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
