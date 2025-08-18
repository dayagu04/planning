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

class SLTPoint {
 public:
  SLTPoint() = default;
  SLTPoint(const double s, const double l, const double t);

  double x() const = delete;
  double y() const = delete;

  double s() const;
  double l() const;
  double t() const;
  void set_s(const double s);
  void set_l(const double s);
  void set_t(const double t);

 protected:
  double s_ = 0.0;
  double l_ = 0.0;
  double t_ = 0.0;
};

}  // namespace planning