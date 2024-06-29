/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#ifndef QR_VISUALIZATION_H
#define QR_VISUALIZATION_H

#include <ostream>
#include <string>
#include <vector>

class StatisticAnalysis {
public:
  StatisticAnalysis(float mean = 0.f);
  ~StatisticAnalysis() = default;

  void Update(float in);
  void Reset(float mean = 0.f);
  float GetMean();
  float GetStandradVar();
  void PrintStatistics();
  float defaultMean;
  float mean_;
  float var_;
  float sigma_;
  float sum_;
  int num;
  std::vector<float> data;
};

class Visualization2D {
public:
  Visualization2D();
  ~Visualization2D() = default;

  void Show();
  void SetLabelNames(std::vector<std::string>);

  std::vector<float> datax;
  std::vector<float> datay1, datay2, datay3, datay4, datay5, datay6;
  std::vector<std::string> labelNames;
  StatisticAnalysis sa[6];
};

#endif  // QR_VISUALIZATION_H
