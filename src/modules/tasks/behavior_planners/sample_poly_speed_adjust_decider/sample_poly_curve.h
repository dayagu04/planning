#pragma once

namespace planning {
class SamplePolyCurve {
 public:
  SamplePolyCurve() = default;
  virtual ~SamplePolyCurve() = default;
  virtual double CalcS(const double t) const = 0;
  virtual double CalcV(const double t) const = 0;
  virtual double CalcAcc(const double t) const = 0;
  virtual double CalcJerk(const double t) const = 0;

  double arrived_t() const { return arrived_t_; }
  double arrived_s() const { return arrived_s_; }
  double arrived_v() const { return arrived_v_; }

  double mid_s() const { return mid_s_; }
  double mid_t() const { return mid_t_; }
  double mid_v() const { return mid_v_; }

  void set_safe_valid(const bool safe_valid) { safe_valid_ = safe_valid; };

 public:
  double arrived_s_;
  double arrived_t_;
  double arrived_v_;
  double arrived_a_ = 0.0;
  double mid_s_;
  double mid_t_;
  double mid_v_;
  double front_edge_to_rear_axle_;
  double back_edge_to_rear_axle_;

  double cost_sum_ = 0.0;
  bool safe_valid_{false};
  bool gap_valid_{true};
};

struct SamplePolyCurveConfig {
  double jerk_limit_upper_ = 0.0;
  double jerk_limit_lower_ = 0.0;
  double acc_limit_upper_ = 0.0;
  double acc_limit_lower_ = 0.0;
  double vel_limit_upper_ = 0.0;
  double vel_limit_lower_ = 0.0;
};
}  // namespace planning