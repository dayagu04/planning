#pragma once

#include <bits/stdint-intn.h>
#include "log_glog.h"

namespace planning {

struct SVPoint {
  // [0, inf]
  double s = 0.0;
  // [0, inf]
  double v = 0.0;
  //[-inf, inf]
  double acc = 0.0;
  double jerk = 0.0;

  double t = 0.0;

  SVPoint() = default;

  SVPoint(const double s_, const double v_) : s(s_), v(v_), acc(0) {}

  SVPoint(const double s_, const double v_, const double acc_)
      : s(s_), v(v_), acc(acc_) {}

  SVPoint(const double s_, const double v_, const double acc_,
          const double jerk_)
      : s(s_), v(v_), acc(acc_), jerk(jerk_) {}

  void DebugString() const {
    ILOG_INFO << "s = " << s << ", v = " << v << ", acc = " << acc;
    return;
  }
};

struct SVGridIndex {
  int32_t s_index;
  int32_t v_index;

  SVGridIndex() = default;

  SVGridIndex(const int32_t s, const int32_t v) : s_index(s), v_index(v) {}
};

struct SpeedBoundary {
  double upper;
  double lower;

  void DebugString() const {
    ILOG_INFO << "v upper = " << upper << ",lower = " << lower;
    return;
  }
};

struct SpeedBoundaryIndex {
  int32_t upper_index;
  int32_t lower_index;

  void DebugString() const {
    ILOG_INFO << "v upper id = " << upper_index
              << ",lower id = " << lower_index;
    return;
  }
};

}  // namespace planning