#pragma once

#include <vector>

#include "st_spline.h"

namespace planning {

struct DvPoint {
  double d;
  double v;
};

class DvSpline {
 public:
  DvSpline() = default;
  ~DvSpline() = default;
  void sort_by_d();
  std::vector<DvPoint>* get_mutable_dv_points() { return &dv_points_; }
  const std::vector<DvPoint>& get_dv_points() const { return dv_points_; }

 private:
  std::vector<DvPoint> dv_points_;
};

}  // namespace planning