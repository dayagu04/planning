#include "speed_data.h"
#include "src/modules/common/math/linear_interpolation.h"
#include "src/modules/common/math/math_utils.h"
namespace planning {

SpeedData::SpeedData(std::vector<SpeedInfo> speed_points)
    : std::vector<SpeedInfo>(std::move(speed_points)) {
  std::sort(begin(), end(), [](const SpeedInfo& p1, const SpeedInfo& p2) {
    return p1.t < p2.t;
  });
}

bool SpeedData::EvaluateByTime(const double t,
                               SpeedInfo* const speed_point) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().t < t + 1.0e-6 && t - 1.0e-6 < back().t)) {
    return false;
  }

  auto comp = [](const SpeedInfo& sp, const double t) {
    return sp.t < t;
  };

  auto it_lower = std::lower_bound(begin(), end(), t, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double t0 = p0.t;
    double t1 = p1.t;

    speed_point->t = t;
    double target_s = planning_math::lerp(p0.s, t0, p1.s, t1, t);
    speed_point->s = target_s;
    double target_l = planning_math::lerp(p0.l, t0, p1.l, t1, t);
    speed_point->l = target_l;
    double target_v = planning_math::lerp(p0.v, t0, p1.v, t1, t);
    speed_point->v = target_v;
    double target_a = planning_math::lerp(p0.a, t0, p1.a, t1, t);
    speed_point->a = target_a;
    double target_da = planning_math::lerp(p0.da, t0, p1.da, t1, t);
    speed_point->da = target_da;
  }
  return true;
}

bool SpeedData::EvaluateByS(const double s,
                            SpeedInfo* const speed_point) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().s < s + 1.0e-6 && s - 1.0e-6 < back().s)) {
    return false;
  }

  auto comp = [](const SpeedInfo& sp, const double s) {
    return sp.s < s;
  };

  auto it_lower = std::lower_bound(begin(), end(), s, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double s0 = p0.s;
    double s1 = p1.s;

    speed_point->s = s;
    double target_t = planning_math::lerp(p0.t, s0, p1.t, s1, s);
    speed_point->t = target_t;
    double target_l = planning_math::lerp(p0.l, s0, p1.l, s1, s);
    speed_point->l = target_l;
    double target_v = planning_math::lerp(p0.v, s0, p1.v, s1, s);
    speed_point->v = target_v;
    double target_a = planning_math::lerp(p0.a, s0, p1.a, s1, s);
    speed_point->a = target_a;
    double target_da = planning_math::lerp(p0.da, s0, p1.da, s1, s);
    speed_point->da = target_da;
  }
  return true;
}

double SpeedData::TotalTime() const {
  if (empty()) {
    return 0.0;
  }
  return back().t - front().t;
}

double SpeedData::TotalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().s - front().s;
}

}