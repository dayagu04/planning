#pragma once

#include <utility>
#include <vector>

#include "log_glog.h"

namespace planning {
class SpeedLimitProfile {
 public:
  SpeedLimitProfile() = default;

  void AppendSpeedLimit(const double s, const double v);

  const std::vector<std::pair<double, double>>& SpeedLimitPoints() const;

  double GetSpeedLimitByS(const double s) const;

  void SetSpeedLimitByIndex(const int index, const double v);

  void Clear();

  const size_t GetIndexByS(const double s);

  // return lowest speed
  double GetSpeedLimitByRange(const double range_start_s,
                              const double range_end_s) const;

 private:
  void DebugString();

 private:
  // use a vector to represent speed limit
  // the first number is s, the second number is v
  // It means at distance s from the start point, the speed limit is v.
  // std::pair<double, double>: s, v
  std::vector<std::pair<double, double>> speed_limit_points_;
};

}  // namespace planning
