#pragma once

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <assert.h>
#include "config/basic_type.h"
#include "define/geometry.h"
#include "session.h"
#include "task_basic_types.h"
namespace planning {
using namespace planning_math;

struct SpeedInfo {
  double s = 0.0;
  double l = 0.0;
  double t = 0.0;
  // speed (m/s)
  double v = 0.0;
  // acceleration (m/s^2)
  double a = 0.0;
  // jerk (m/s^3)
  double da = 0.0;
};

class SpeedData : public std::vector<SpeedInfo> {
 public:
  SpeedData() = default;

  virtual ~SpeedData() = default;

  explicit SpeedData(std::vector<SpeedInfo> speed_points);

  void AppendSpeedPoint(const double s, const double l, const double time,
                        const double v, const double a, const double da);

  bool EvaluateByTime(const double time, SpeedInfo* const speed_point) const;

  // Assuming spatial traversed distance is monotonous, which is the case for
  // current usage on city driving scenario
  bool EvaluateByS(const double s, SpeedInfo* const speed_point) const;

  double TotalTime() const;

  // Assuming spatial traversed distance is monotonous
  double TotalLength() const;
};

}  // namespace planning