#pragma once
#include "array"
#include "sample_poly_const.h"
#include "sample_poly_curve.h"
#include "trajectory1d/second_order_time_optimal_trajectory.h"
#include "vector"

namespace planning {
struct UniformJerkCurveCoffi {
  std::vector<std::array<double, 4>> cofficients;
  std::vector<double> time_points;
  void clear() {
    cofficients.clear();
    time_points.clear();
  }
};

class UniformJerkCurve : public SamplePolyCurve {
 public:
  UniformJerkCurve() = default;
  UniformJerkCurve(const StateLimit& state_limit, const LonState& lon_state,
                   bool is_upper_limit);
  ~UniformJerkCurve() = default;

  double CalcS(const double t) const override;
  double CalcV(const double t) const override;
  double CalcAcc(const double t) const override;
  double CalcJerk(const double t) const override;

  UniformJerkCurveCoffi GetCoffi() const { return jerk_curve_coffi_; }
  LonState GetLonState() const { return lon_state_; }
  StateLimit GetStateLimit() const { return state_limit_; }
  void BuildUniformJerkTraj();

 private:
  LonState lon_state_;
  StateLimit state_limit_;
  UniformJerkCurveCoffi jerk_curve_coffi_;
};
}  // namespace planning