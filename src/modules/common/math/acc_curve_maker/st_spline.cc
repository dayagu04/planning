#include "st_spline.h"

#include <algorithm>

namespace planning {

StPoint StSpline::get_point_by_t(double t) {
  StPoint output;
  if (st_points_.size() == 0) {
    return output;
  }
  int head = 0, tail = st_points_.size() - 1;
  for (int i = 0; i < st_points_.size(); ++i) {
    if (tail - head == 1) {
      double rate =
          (t - st_points_[head].t) / (st_points_[tail].t - st_points_[head].t);
      output.t = t;
      output.s =
          st_points_[head].s + rate * (st_points_[tail].s - st_points_[head].s);
      output.v =
          st_points_[head].v + rate * (st_points_[tail].v - st_points_[head].v);
      output.a =
          st_points_[head].a + rate * (st_points_[tail].a - st_points_[head].a);
      output.j =
          st_points_[head].j + rate * (st_points_[tail].j - st_points_[head].j);
      return output;
    } else {
      int mid = (head + tail) / 2;
      if (st_points_[mid].t >= t) {
        tail = mid;
      } else {
        head = mid;
      }
    }
  }

  return output;
}

StPoint StSpline::get_point_by_s(double s) {
  StPoint output;
  if (st_points_.size() == 0) {
    return output;
  }
  int head = 0, tail = st_points_.size() - 1;
  for (int i = 0; i < st_points_.size(); ++i) {
    if (tail - head == 1) {
      double rate =
          (s - st_points_[head].s) / (st_points_[tail].s - st_points_[head].s);
      output.s = s;
      output.t =
          st_points_[head].t + rate * (st_points_[tail].t - st_points_[head].t);
      output.v =
          st_points_[head].v + rate * (st_points_[tail].v - st_points_[head].v);
      output.a =
          st_points_[head].a + rate * (st_points_[tail].a - st_points_[head].a);
      output.j =
          st_points_[head].j + rate * (st_points_[tail].j - st_points_[head].j);
      return output;
    } else {
      int mid = (head + tail) / 2;
      if (st_points_[mid].s >= s) {
        tail = mid;
      } else {
        head = mid;
      }
    }
  }

  return output;
}

void StSpline::sort_by_t() {
  auto comp = [](const StPoint& p1, const StPoint& p2) { return p1.t < p2.t; };
  std::sort(st_points_.begin(), st_points_.end(), comp);
}

void StSpline::sort_by_s() {
  auto comp = [](const StPoint& p1, const StPoint& p2) { return p1.s < p2.s; };
  std::sort(st_points_.begin(), st_points_.end(), comp);
}

}  // namespace planning