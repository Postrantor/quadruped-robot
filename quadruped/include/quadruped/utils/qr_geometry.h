/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_GEOMETRY_H
#define QR_GEOMETRY_H

#include <cmath>
#include <iostream>
#include <type_traits>

#include "utils/qr_algebra.h"
#include "utils/qr_cpptypes.h"
#include "utils/qr_se3.h"

namespace robotics {

namespace math {

/**
 * @brief 在状态空间中线性插值两个点。
 * @example
 *  robotics::math::Segment<float, Vec6<float>> segment;
 *  Vec6<float> s = Vec6<float>::Zero();
 *  Vec6<float> t;
 *  t<< 1, 1, 1, 1, 0, 0;
 *  segment.Reset(s,t);
 *  Vec6<float> midpoint = segment.GetPoint(0.9);
 *  cout << midpoint << endl;
 *
 */
template <class t = float, class T = Vec6<t>>
class qrSegment {
public:
  /**
   * @brief qrSegment类的构造函数。
   */
  qrSegment() = default;

  qrSegment(T source_, T dest_) {
    source = source_;
    dest = dest_;
  }

  /**
   * @brief qrSegment类的析构函数。
   */
  ~qrSegment() = default;

  T GetPoint(float phase) {
    if (phase > 1.0) {
      phase = 1.0;
    } else if (phase < 0.0) {
      phase = 0.0;
    }
    T point = phase * dest + (1.0 - phase) * source;
    return point;
  }

  void Reset(T source_, T dest_) {
    source = source_;
    dest = dest_;
  }

private:
  T source;

  T dest;
};

/**
 * @ref https://math.stackexchange.com/questions/2217654/interpolation-in-so3-different-approaches
 * @ref
 * https://math.stackexchange.com/questions/1832019/what-is-the-angular-velocity-in-an-inertial-frame-given-the-angular-velocity-in
 *
 */
class qrCubicSplineInSO3 {
public:
  /**
   * @brief qrCubicSplineInSO3类的构造函数。
   */
  qrCubicSplineInSO3() = default;

  qrCubicSplineInSO3(Mat3<float> R1, Mat3<float> R2, float t0, float duration, Mat3<float> inertial);

  /**
   * @brief qrCubicSplineInSO3类的析构函数。
   */
  ~qrCubicSplineInSO3() = default;

  void GetPoint(float currentTime, Mat3<float> &outR, Vec3<float> &outwb);

private:
  float t0;
  float dt;
  float lastTime;
  Mat3<float> R1;
  Mat3<float> lastR;
  Mat3<float> R2;
  Mat3<float> dR1;
  Mat3<float> dR2;
  Mat3<float> M0;
  Mat3<float> M1;
  Mat3<float> M2;
  Mat3<float> M3;
  Mat3<float> W;
};

/**
 * @brief Spline类定义了一个抽象类，用于不同的样条插值。
 */
class qrSpline {
public:
  /**
   * @brief 记录空间信息，包括时间导数。
   */
  struct Point {
    /**
     * @brief 结构体Point的构造函数。
     */
    Point() : x(0.0), xd(0.0), xdd(0.0) {}

    Point(float p, float v = 0.0, float a = 0.0) : x(p), xd(v), xdd(a) {}

    void setZero() {
      x = 0.0;
      xd = 0.0;
      xdd = 0.0;
    }

    friend std::ostream &operator<<(std::ostream &cout, Point &p);

    float x;
    float xd;
    float xdd;
  };

  qrSpline::Point operator=(const qrSpline::Point &rhs) {
    qrSpline::Point out;
    out.x = rhs.x;
    out.xd = rhs.xd;
    out.xdd = rhs.xdd;
    return out;
  }

  /**
   * @brief Spline类的构造函数。
   */
  qrSpline() : initial_time_(0.), duration_(0.) {}

  /**
   * @brief Spline类的构造函数。
   * @param initial_time: 初始时间
   * @param duration:样条的持续时间
   * @param start: 起点
   * @param end: 终点
   */
  qrSpline(const float &initial_time, const float &duration, const Point &start, const Point &end);

  /**
   * @brief Spline 类的构造函数。
   * @param initial_time: 初始时间
   * @param duration: Spline 的持续时间
   * @param start_p: 起始点
   * @param end_p: 终点
   */
  qrSpline(const float &initial_time, const float &duration, float start_p, float end_p);

  /**
   * @brief Spline 类的析构函数。
   */
  virtual ~qrSpline() = default;

  /**
   * @brief 设置 Spline 的边界
   * @param initial_time: 初始时间
   * @param duration: Spline 的持续时间
   * @param start_p: 起始点
   * @param end_p: 终点
   */
  void setBoundary(const float &initial_time, const float &duration, const Point &start_p, const Point &end_p);

  /**
   * @brief 设置 Spline 的边界
   * @param initial_time: 初始时间
   * @param duration: Spline 的持续时间
   * @param start_p: 起始点
   * @param end_p: 终点
   */
  void setBoundary(const float &initial_time, const float &duration, const float &start_p, const float &end_p);

  /**
   * @brief 根据 Spline 插值获取点的值
   * @param current_time: 当前时间
   * @param p: 点值
   */
  virtual bool getPoint(const float &current_time, Point &p) = 0;

  /**
   * @brief 根据 Spline 插值获取点的值
   * @param current_time: 当前时间
   * @param p: 点值
   */
  virtual bool getPoint(const float &current_time, float &p) = 0;

  bool isTimeElapsed(float &time);

protected:
  /** @brief Spline 的初始时间 */
  float initial_time_;

  /** @brief Spline 的持续时间 */
  float duration_;

  /** Spline 的起始点 */
  Point start_;

  /** @brief Spline 的终点 */
  Point end_;
};

inline bool qrSpline::isTimeElapsed(float &t) {
  /* 这只在时间间隔之外有效 */
  if ((t - initial_time_) > duration_)
    return true;
  else
    return false;
}

class qrQuadraticSpline : public qrSpline {
public:
  /**
   * @brief 二次 Spline 的构造函数。
   */
  qrQuadraticSpline() = default;

  /**
   * @brief 二次 Spline 的构造函数。
   * @param initial_time: 初始时间
   * @param duration: Spline 的持续时间
   * @param start: 起始点
   * @param end: 终点
   */
  qrQuadraticSpline(const float &initial_time, const float &duration, const Point &start, const Point &end)
      : qrSpline(initial_time, duration, start, end) {}

  /**
   * @brief 二次 Spline 的构造函数。
   * @param initial_time: 初始时间
   * @param duration: Spline 的持续时间
   * @param start: 起始点
   * @param end: 终点
   */
  qrQuadraticSpline(const float &initial_time, const float &duration, float start, float end)
      : qrSpline(initial_time, duration, start, end) {}

  /**
   * @brief 二次 Spline 的析构函数。
   */
  ~qrQuadraticSpline() = default;

  /**
   * @brief 根据 Spline 插值获取点的值
   * @param current_time: 当前时间
   * @param mid: 中间点值
   * @param p: 点值
   */
  bool getPoint(const float &current_time, float mid, Point &p);

  virtual bool getPoint(const float &current_time, Point &p);

  /**
   * @brief 根据 Spline 插值获取点的值
   * @param current_time: 当前时间
   * @param mid: 中间点值
   * @param p: 点值
   */
  bool getPoint(const float &current_time, float mid, float &p);

  virtual bool getPoint(const float &current_time, float &p);
};

/**
 * @brief 三次样条插值类
 */
class qrCubicSpline : public qrSpline {
public:
  /**
   * @brief 构造函数
   */
  qrCubicSpline() = default;

  /**
   * @brief 构造函数
   * @param initial_time: 初始时间
   * @param duration: 插值持续时间
   * @param start: 起始点
   * @param end: 终点
   */
  qrCubicSpline(const float &initial_time, const float &duration, const Point &start, const Point &end)
      : qrSpline(initial_time, duration, start, end) {}

  /**
   * @brief 构造函数
   * @param initial_time: 初始时间
   * @param duration: 插值持续时间
   * @param start: 起始点
   * @param end: 终点
   */
  qrCubicSpline(const float &initial_time, const float &duration, float start, float end)
      : qrSpline(initial_time, duration, start, end) {}

  /**
   * @brief 析构函数
   */
  ~qrCubicSpline() = default;

  /**
   * @brief 根据插值获取点值
   * @param current_time: 当前时间
   * @param p: 点值
   */
  bool getPoint(const float &current_time, Point &p);

  /**
   * @brief 根据插值获取点值
   * @param current_time: 当前时间
   * @param p: 点值
   */
  bool getPoint(const float &current_time, float &p);
};

/**
 * @brief 五次多项式插值类
 */
class qrFifthOrderPolySpline : public qrSpline {
public:
  /**
   * @brief 构造函数
   */
  qrFifthOrderPolySpline() = default;

  /**
   * @brief 构造函数
   * @param initial_time: 初始时间
   * @param duration: 插值持续时间
   * @param start: 起始点
   * @param end: 终点
   */
  qrFifthOrderPolySpline(const float &initial_time, const float &duration, const Point &start, const Point &end)
      : qrSpline(initial_time, duration, start, end) {}

  /**
   * @brief 析构函数
   */
  ~qrFifthOrderPolySpline() = default;

  /**
   * @brief 根据插值获取点值
   * @param current_time: 当前时间
   * @param p: 点值
   */
  bool getPoint(const float &current_time, Point &p);

  /**
   * @brief 根据插值获取点值
   * @param current_time: 当前时间
   * @param p: 点值
   */
  bool getPoint(const float &current_time, float &p);
};

/**
 * @brief 线性插值类
 */
class qrLinearSpline : public qrSpline {
public:
  /**
   * @brief 构造函数
   */
  qrLinearSpline() = default;

  /**
   * @brief 构造函数
   * @param initial_time: 初始时间
   * @param duration: 插值持续时间
   * @param start: 起始点
   * @param end: 终点
   */
  qrLinearSpline(const float &initial_time, const float &duration, const Point &start, const Point &end)
      : qrSpline(initial_time, duration, start, end) {}

  /**
   * @brief 析构函数
   */
  ~qrLinearSpline() = default;

  /**
   * @brief 根据插值获取点值
   * @param current_time: 当前时间
   * @param p: 点值
   */
  bool getPoint(const float &current_time, Point &p);

  /**
   * @brief 根据插值获取点值
   * @param current_time: 当前时间
   * @param p: 点值
   */
  bool getPoint(const float &current_time, float &p);
};

/**
 * @brief 线性插值
 */
template <typename y_t, typename x_t>
y_t lerp(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value, "must use floating point value");
  assert(x >= 0 && x <= 1);
  return y0 + (yf - y0) * x;
}

/**
 * @brief 三次贝塞尔插值
 */
template <typename y_t, typename x_t>
y_t cubicBezier(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value, "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = x * x * x + x_t(3) * (x * x * (x_t(1) - x));
  return y0 + bezier * yDiff;
}

/**
 * @brief 三次贝塞尔插值一阶导数
 */
template <typename y_t, typename x_t>
y_t cubicBezierFirstDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value, "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) * x * (x_t(1) - x);
  return bezier * yDiff;
}

/**
 * @brief 三次贝塞尔插值二阶导数
 */
template <typename y_t, typename x_t>
y_t cubicBezierSecondDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value, "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) - x_t(12) * x;
  return bezier * yDiff;
}

}  // Namespace math

}  // Namespace robotics

#endif  // QR_GEOMETRY_H
