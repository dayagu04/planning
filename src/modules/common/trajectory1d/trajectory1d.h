#pragma once

#include <cstdint>

namespace planning {

struct SecondOrderParam {
  double t1 = 0.0;
  double t2 = 0.0;
  double t3 = 0.0;
  double j1 = 0.0;
  double j2 = 0.0;
  double j3 = 0.0;
};

struct ThirdOrderParam {
  SecondOrderParam Pa;
  SecondOrderParam Pb;
  double tc = 0.0;
  double tpb = 0.0;
  double vc = 0.0;
};

struct LonState {
  double p = 0.0;
  double v = 0.0;
  double a = 0.0;
  double j = 0.0;
  double t = 0.0;
};

struct StateLimit {
  double p_end = 0.0;
  double v_end = 0.0;
  double v_min = 0.0;
  double v_max = 0.0;
  double a_min = 0.0;
  double a_max = 0.0;
  double j_min = 0.0;
  double j_max = 0.0;
};

struct CoordinateParam {
  double s_start = 0.0;  // s_start is target s(location) at current time
  double v = 0.0;        // v is target v at current time
};

class Trajectory1d {
 public:
  Trajectory1d() = default;
  Trajectory1d(const Trajectory1d& other) = default;
  virtual ~Trajectory1d() = default;

  virtual double Evaluate(const int32_t order, const double param) const = 0;
  virtual double ParamLength() const = 0;
};

}  // namespace planning
