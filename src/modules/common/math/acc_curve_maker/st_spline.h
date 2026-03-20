#pragma once

#include <vector>

namespace planning {

struct StPoint {
  double t;
  double s;
  double v;
  double a;
  double j;
};

class StSpline {
 public:
  const std::vector<StPoint>& get_st_points() const { return st_points_; }
  std::vector<StPoint>* get_mutable_st_points() { return &st_points_; }
  StPoint get_point_by_t(double t);
  StPoint get_point_by_s(double s);
  void sort_by_t();
  void sort_by_s();

 private:
  std::vector<StPoint> st_points_;
};

}  // namespace planning